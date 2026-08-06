module BalansBot

using BalansBotDrivers
using BalansBotDrivers.GPIO
using BalansBotDrivers.PWM
using BalansBotDrivers.SPI
import BalansBotDrivers.ICM42688PC
import BalansBotDrivers.DAC43701
import BalansBotDrivers.Motor
using PIOLib
using Configurations
using StatsBase
using Libevdev
using PrecompileTools
using Dates

using ROSNode

module Msgs
    using ROSNode
    @ros_package "balans_msgs"
    @ros_message struct EyesValue; left::Int; right::Int; end
    @ros_message struct NoseColor; r::Int; g::Int; b::Int; end
    @ros_message struct WheelPosition; r::Int; l::Int end
    @ros_message struct WheelDrive; r::Float64; l::Float64 end
    @ros_message struct ControllerUpdate
        angle::Float64
        pwm_left::Float64
        pwm_right::Float64
        innovation::Float64
        speed::Float64
        balance_control::Float64
        speed_control::Float64
    end
    @ros_message struct InertialMeasurement 
        accel_x::Float64
        accel_y::Float64
        accel_z::Float64
        temp::Float64
        gyro_x::Float64
        gyro_y::Float64
        gyro_z::Float64
    end
    @ros_message struct BotRunning
        balancing::Bool
    end
    @ros_message struct BotParameters
        angle_zero::Float32
        angular_velocity_zero::Float32
        kp_balance::Float32
        kd_balance::Float32
        kp_speed::Float32
        ki_speed::Float32
        kp_turn::Float32
        kd_turn::Float32
    end
end

@option struct DyadBotOptions
    gyro_bias::Union{Nothing, Vector{Float64}} = nothing
    calibration_data::Union{Nothing, Vector{UInt8}} = nothing
    threshold_low::Float64 = 0.8
    threshold_high::Float64 = 3.0
end

@component mutable struct BalansCore{Name} <: Component{Name}
    gpio::Union{Nothing, GPIO.GPIOController} = nothing
    cm_present::Union{Nothing, GPIO.GPIOPin} = nothing
    d1::Union{Nothing, GPIO.GPIOPin} = nothing
    d2::Union{Nothing, GPIO.GPIOPin} = nothing
    pins::Vector{GPIO.GPIOPin}=[]

    function configure(node, b::BalansCore)
        b.gpio = GPIO.open_gpio("/dev/gpiochip0")
        b.cm_present = GPIO.request_output(b.gpio, BalansBotDrivers.CM_PRESENT, "cm_present", 0)
        b.d1 = GPIO.request_output(b.gpio, BalansBotDrivers.D1, "d1", 0)
        b.d2 = GPIO.request_output(b.gpio, BalansBotDrivers.D2, "d2", 0)
        GPIO.set_value(b.cm_present, 1)
        b.pins = [b.cm_present, b.d1, b.d2]
        println(Core.stdout, "GPIO configured")
    end

    function cleanup(node, b::BalansCore)
        for pin in b.pins
            close(pin)
        end
        close(b.gpio)
    end
end

@component mutable struct PIOControl{Name} <: Component{Name}
    pio::Union{Nothing, PIOBlock} = nothing
    configure(node, p::PIOControl) = p.pio = open_pio(0)
    cleanup(node, p::PIOControl) = close(p.pio)
end

@component mutable struct ShiftRegisters{Name} <: Component{Name}
    @requires pioc::PIOControl
    chain::Union{Nothing, ShiftRegisterChain} = nothing
    
    function configure(node, p::ShiftRegisters) 
        p.chain = open_shift_registers(p.pioc.pio)
        p.chain[0:23] = false
        println(Core.stdout, "Shift registers initialized")
    end
    function cleanup(node, p::ShiftRegisters) 
        p.chain[0:23] = false
        close(p.chain)
    end
end

@component mutable struct NoseControl{Name} <: Component{Name}
    @requires pioc::PIOControl
    nose::Union{Nothing,BalansBotDrivers.WS2812} = nothing
    @hears function hears(node, e::NoseControl, dsp::Msgs.NoseColor)
        set_color!(p.nose, dsp.r, dsp.g, dsp.b)
    end
    function configure(node, p::NoseControl) 
        p.nose = BalansBotDrivers.open_ws2812(p.pioc.pio, BalansBotDrivers.NOSE_RGB)
        set_color!(p.nose, 255, 0, 0)
    end
    function cleanup(node, p::NoseControl) 
        set_color!(p.nose, 0, 0, 0)
        close(p.nose)
    end

end

@component mutable struct EyesControl{Name} <: Component{Name}
    @requires shifts::ShiftRegisters
    disp1::Union{Nothing,BalansBotDrivers.SevenSeg}=nothing
    disp2::Union{Nothing,BalansBotDrivers.SevenSeg}=nothing
    @hears function eyes(node, e::EyesControl, dsp::Msgs.EyesValue)
        BalansBotDrivers.transaction(e.shifts.chain) do
            BalansBotDrivers.show_digit!(e.disp2, dsp.left)
            BalansBotDrivers.show_digit!(e.disp1, dsp.right)
        end
    end
    function configure(node, p::EyesControl) 
        p.disp1 = BalansBotDrivers.SevenSeg(p.shifts.chain, 8)   # 2nd register: bits 8–15
        p.disp2 = BalansBotDrivers.SevenSeg(p.shifts.chain, 16)  # 3rd register: bits 16–23
    end
    function cleanup(node, p::EyesControl) 
        BalansBotDrivers.clear!(p.disp2)
        BalansBotDrivers.clear!(p.disp1)
    end
end


@component mutable struct BotOptions{Name} <: Component{Name}
    options::Union{DyadBotOptions, Nothing} = nothing
    function configure(node, p::BotOptions)
        if !isfile("options.toml")
            to_toml("options.toml", DyadBotOptions(nothing, nothing, 0.8, 3.0); include_defaults=true)
        end
        p.options = from_toml(DyadBotOptions, "options.toml")
    end
end
function save_options(p::BotOptions)
    to_toml("options.toml", p.options; include_defaults=true)
end


@component mutable struct IMUDriver{Name} <: Component{Name}
    @requires shifts::ShiftRegisters
    @requires opts::BotOptions
    imu::Union{ICM42688PC.ICM42688PC, Nothing} = nothing
    @publishes imu_measurement::Msgs.InertialMeasurement
    @every 200 function tick(node, m::IMUDriver)
        state = ICM42688PC.read_all(m.imu)
        publish(entities(node, m).imu_measurement, Msgs.InertialMeasurement(state...))
    end
    
    function configure(node, p::IMUDriver)
        for i=1:100
            sleep(0.0001)
            BalansBotDrivers.transaction(p.shifts.chain) do
                p.shifts.chain[3]=false
                p.shifts.chain[7]=true
            end
        end
        options = p.opts.options
        imu = ICM42688PC.ICM42688PC(0,0; speed_hz=UInt32(15_000_000))
        ICM42688PC.soft_reset!(imu)
        good_who_am_i = ICM42688PC.check_who_am_i(imu)
        println(Core.stdout, "IMU who_am_i good? = $(good_who_am_i)")
        good_self_test = ICM42688PC.accel_self_test(imu) && ICM42688PC.gyro_self_test(imu)
        println(Core.stdout, "IMU self test good? = $(good_self_test)")
        if isnothing(options.calibration_data)
            println(Core.stdout, "No saved IMU gains. Keep the bot still and hit enter.")
            readline()
            cod_result, gains = ICM42688PC.calibration_on_demand(imu)
            println(Core.stdout, "IMU CoD success = $(!cod_result.COD_Failed); gains = $gains. Calibrating gyro bias.")
            gx = []; gy = []; gz = []
            ICM42688PC.set_gyro_config!(imu, ICM42688PC.Registers.GYRO_FS_512DPS, ICM42688PC.Registers.GYRO_ODR_896_8HZ)
            ICM42688PC.set_accel_config!(imu, ICM42688PC.Registers.ACCEL_FS_2G, ICM42688PC.Registers.ACCEL_ODR_1000HZ)
            ICM42688PC.set_sensor_config!(imu, true, true, false, false, false)
            sleep(0.1)
            for i=1:100
                m =ICM42688PC.read_all(imu)
                push!(gx, m.gyro_x); push!(gy, m.gyro_y); push!(gz, m.gyro_z)
                sleep(0.01)
            end
            ICM42688PC.set_sensor_config!(imu, false, false, false, false, false)
            offsets = (mean(gx), mean(gy), mean(gz))

            println(Core.stdout, "IMU calibration complete")
            options.calibration_data = gains
            options.gyro_bias = collect(offsets)
            ICM42688PC.set_gyro_bias(imu, offsets)
        else
            println(Core.stdout, "Loading saved IMU gains.")
            ICM42688PC.load_calibration(imu, options.calibration_data)
            ICM42688PC.set_gyro_bias(imu, (options.gyro_bias...,))
        end 
        ICM42688PC.set_gyro_config!(imu, ICM42688PC.Registers.GYRO_FS_512DPS, ICM42688PC.Registers.GYRO_ODR_896_8HZ)
        ICM42688PC.set_accel_config!(imu, ICM42688PC.Registers.ACCEL_FS_2G, ICM42688PC.Registers.ACCEL_ODR_1000HZ)
        ICM42688PC.set_lpf_config!(imu, ICM42688PC.Registers.LPF_MODE_13_37, ICM42688PC.Registers.LPF_MODE_13_37)
        ICM42688PC.set_sensor_config!(imu, true, true, false, false, false)
        p.imu = imu
        save_options(p.opts)
        println(Core.stdout, "IMU initialized!")
    end
    function cleanup(node, p::IMUDriver)
        ICM42688PC.close!(p.imu)
    end
end


@component mutable struct BalansArbiter{Name} <: Component{Name}
    running::Bool = true
    @publishes balancing::Msgs.BotRunning
    @hears function balance(node, a::BalansArbiter, dsp::Msgs.BotRunning) # todo: qos
        a.running = dsp.balancing
    end
    @every 20 function tick(node, a::BalansArbiter)
        publish(entities(node, a).balancing, Msgs.BotRunning(a.running))
    end
end

@component mutable struct Wheels{Name} <: Component{Name}
    @requires shifts::ShiftRegisters
    @requires options::BotOptions
    @requires arbiter::BalansArbiter
    @param max_command::Float64 = 0.8
    Vthp::Union{DAC43701.DACDevice, Nothing} = nothing
    Vthm::Union{DAC43701.DACDevice, Nothing} = nothing
    chip::Union{PWMChip, Nothing} = nothing
    motor_a::Union{Motor.Motor, Nothing} = nothing
    motor_b::Union{Motor.Motor, Nothing} = nothing

    @publishes rotation::Msgs.WheelPosition
    @hears function balancing(node, e::Wheels, dsp::Msgs.BotRunning)
        if !dsp.balancing
            Motor.set_drive!(e.motor_a, 0.0)
            Motor.set_drive!(e.motor_b, 0.0) 
        end
    end
    @hears function wheel_drive(node, e::Wheels, dsp::Msgs.WheelDrive)
        if e.arbiter.running
            Motor.set_drive!(e.motor_a, clamp(dsp.l, -parameters(node, e).max_command, parameters(node, e).max_command))
            Motor.set_drive!(e.motor_b, clamp(dsp.r, -parameters(node, e).max_command, parameters(node, e).max_command)) 
        else
            Motor.set_drive!(e.motor_a, 0.0)
            Motor.set_drive!(e.motor_b, 0.0) 
        end
    end
    function configure(node, p::Wheels)
        # ADS7142: 0x18
        # Vth+ - GPI1 = chain[4] - 0x48
        # Vth- - GPI2 = chain[5] - 0x49
        # Window comparator initialization
        # first check to see if the DACs have been set up already
        (Vthp, Vthm) = if true #!DAC43701.probe_check(1, 0x48) || !DAC43701.probe_check(1, 0x49)
            set_gpi(gpi_no) = function (cb)
                p.shifts.chain[gpi_no] = true
                try
                    cb()
                finally
                    p.shifts.chain[gpi_no] = false
                end
            end
            devs = DAC43701.set_dac_addresses(1, [
                0x48 => set_gpi(4),
                0x49 => set_gpi(5)
            ])
            (devs[0x48], devs[0x49])
        else
            (DAC43701.open_dac(1, 0x48), DAC43701.open_dac(1, 0x49))
        end
        DAC43701.set_dac_code(Vthp, floor(Int, 256 * (p.options.options.threshold_high / 3.3)))
        DAC43701.power_up(Vthp);
        DAC43701.set_dac_code(Vthm, floor(Int, 256 * (p.options.options.threshold_low / 3.3)))
        DAC43701.power_up(Vthm)
        p.Vthp = Vthp
        p.Vthm = Vthm

        p.chip = open_chip()
        p.motor_a = Motor.Motor(export_channel(p.chip, 0), export_channel(p.chip, 1), AxisTracker("/dev/input/event2"), REL_X)
        # flipped since motor_b is the opposite orientation to motor_a
        p.motor_b = Motor.Motor(export_channel(p.chip, 3), export_channel(p.chip, 2), AxisTracker("/dev/input/event1"), REL_Y)
        Motor.enable!(p.motor_a)
        Motor.enable!(p.motor_b)
    end
    
    function cleanup(node, p::Wheels)
        Motor.set_drive!(p.motor_a, 0.0)
        Motor.set_drive!(p.motor_b, 0.0)
        Motor.disable!(p.motor_a)
        Motor.disable!(p.motor_b)
        if !isnothing(p.motor_a) close(p.motor_a) end
        if !isnothing(p.motor_b) close(p.motor_b) end
    end
end

function wheel_angle(node, p::Wheels; should_publish=true)
    angle_a = Motor.angle(p.motor_a)
    angle_b = Motor.angle(p.motor_b)
    pos = Msgs.WheelPosition(angle_a, angle_b)
    if should_publish
        publish(entities(node, p).rotation, pos)
    end
    return angle_a, angle_b
end

@component mutable struct EyesCounting{Name} <: Component{Name}
    last_counter::Int=0
    counter::Int=1
    @publishes eyes::Msgs.EyesValue
    @every 2 function tick(node, e::EyesCounting)
        e.last_counter = e.counter
        e.counter += 1
        if e.counter > 9
            e.counter = 0
        end
        publish(entities(node, e).eyes, Msgs.EyesValue(e.last_counter, e.counter))
    end
end

@component mutable struct EyesSpinning{Name} <: Component{Name}
    @requires wheels::Wheels
    last_counter::Int=0
    counter::Int=1
    @publishes eyes::Msgs.EyesValue
    @every 20 function tick(node, m::EyesSpinning)
        angle_a, angle_b = wheel_angle(node, m.wheels; should_publish=false)
        publish(entities(node, m).eyes, Msgs.EyesValue(mod(div(angle_a, 10), 10), mod(div(angle_b, 10), 10)))
    end
end

module BalanceController
include(joinpath(@__DIR__, "balance_original.jl"))
end

@component mutable struct BalansController{Name} <: Component{Name}
    @requires wheels::Wheels
    @publishes controller_update::Msgs.ControllerUpdate
    @publishes wheel_drive::Msgs.WheelDrive
    controller::BalanceController.BalanceController{BalanceController.ParallelPID} = BalanceController.BalanceController()
    a_last::Int = 0
    b_last::Int = 0
    @hears function bot_params(node, m::BalansController, msg::Msgs.BotParameters)
        m.controller.angle_zero = msg.angle_zero
        m.controller.angular_velocity_zero = msg.angular_velocity_zero
        m.controller.controller.kp_balance = msg.kp_balance
        m.controller.controller.kd_balance = msg.kd_balance
        m.controller.controller.kp_speed = msg.kp_speed
        m.controller.controller.ki_speed = msg.ki_speed
        m.controller.controller.kp_turn = msg.kp_turn
        m.controller.controller.kd_turn = msg.kd_turn
        println("update pars")
    end
    @hears function imu_measurement(node, m, msg::Msgs.InertialMeasurement) 
        angle_a, angle_b = wheel_angle(node, m.wheels)
        kalman_angle, innovation = BalanceController.compute_pwm!(m.controller,
            angle_a - m.a_last, angle_b - m.b_last,
            Float32(msg.accel_x), Float32(-msg.accel_z), Float32(msg.accel_y),
            Float32(msg.gyro_x), Float32(-msg.gyro_z), Float32(msg.gyro_y)) # swap y/z because of the different orientation
        publish(entities(node, m).controller_update, Msgs.ControllerUpdate(; 
            angle = kalman_angle, pwm_left=m.controller.pwm_left, pwm_right=m.controller.pwm_right, innovation=innovation[1], 
            speed=m.controller.controller.speed_filter,
            balance_control=m.controller.controller.balance_control_output,
            speed_control=m.controller.controller.speed_control_output))
        m.a_last = angle_a
        m.b_last = angle_b
        publish(entities(node, m).wheel_drive, Msgs.WheelDrive(-m.controller.pwm_left/255.0, -m.controller.pwm_right/255.0))
    end
end

@component mutable struct WheelTest{Name} <: Component{Name}
    @requires wheels::Wheels
    @publishes wheel_drive::Msgs.WheelDrive
    start::DateTime = now()
    @every 20 function tick(node, m::WheelTest)
        delta = (now() - m.start).value/1000.0
        speed = 0.9 * sin(delta)
        angle_a, angle_b = wheel_angle(node, m.wheels)

        publish(entities(node, m).wheel_drive, Msgs.WheelDrive(speed, speed))
    end
end

const Balans = node(
    "core" => BalansCore, "opts" => BotOptions, "pio" => PIOControl, "shifts" => ShiftRegisters,
    "arbiter" => BalansArbiter, "nose" => NoseControl, "eyes" => EyesControl, "imu" => IMUDriver, 
    "wheels" => Wheels, "eyespinning" => EyesSpinning, #"eyecounter" => EyesCounting, , "wtest" => WheelTest
    "controller" => BalansController;
    name = "balans"
)

greet() = print("Hello World!")

function (@main)(ARGS)
    println("Main app: ", join(ARGS, " "))
    @context(peers = ["tcp/localhost:7447"]) do ctx
        println("starting!")
        balans = run(Balans; ctx=ctx, name="balans", block=false)
        describe_wiring(balans)
        println("spinning!")
        spin(ctx; handle_signals = true)
    end
end

@compile_workload precompile_node(Balans)
@register_nodes Balans
end # module BalansBot
