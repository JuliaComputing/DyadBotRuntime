using StaticArrays
export IMUKalmanFilter, predict!, correct!, update!, compute_angles, bias, angle

# System: x = [angle; bias], dynamics: ẋ = A_c*x + B_c*u, measurement: y = C*x
# Continuous: A_c = [0 -1; 0 0], B_c = [1; 0]
# Discretized with Ts = 0.005 (200 Hz)

const Ts = 0.005f0
const Ad = @SMatrix [1.0f0 -Ts; 0.0f0 1.0f0]   # exp(A_c * Ts), exact since A_c² = 0
const Bd = @SVector [Ts, 0.0f0]             # ∫₀ᵈᵗ exp(A_c*τ) B_c dτ
const Cd = @SMatrix [1.0f0 0.0f0]             # Measurement matrix (1×2)
const I2 = @SMatrix [1.0f0 0.0f0; 0.0f0 1.0f0]    # Identity for covariance update

"""
    IMUKalmanFilter

Discrete-time Kalman filter for angle estimation from gyro (input) and accelerometer (measurement).
State is `[angle, gyro_bias]`.
"""
mutable struct IMUKalmanFilter
    x::SVector{2, Float32}              # State [angle, bias]
    P::SMatrix{2, 2, Float32, 4}        # Covariance
    innovation_threshold::Float32
    const Q::SMatrix{2, 2, Float32, 4}        # Process noise covariance
    const R::SMatrix{1, 1, Float32, 1}        # Measurement noise covariance
end

"""
    IMUKalmanFilter(; Q_angle=0.001f0, Q_bias=0.005f0, R_angle=0.5)

Create a discrete Kalman filter with specified noise covariances.
Default values from BalanceCar.h.
"""
function IMUKalmanFilter(; Q_angle::Float32=0.001f0, Q_bias::Float32=0.005f0, R_angle::Float32=10.0f0, innovation_threshold::Float32=3.84f0)
    x = @SVector zeros(Float32, 2)
    P = I2
    Q = @SMatrix [Q_angle 0.0f0; 0.0f0 Q_bias]
    R = @SMatrix [R_angle;;]
    @show Q_angle R_angle
    IMUKalmanFilter(x, P, innovation_threshold, Q, R)
end

"""
    predict!(kf, u)

Prediction step: propagate state and covariance using gyro tilt rate (x-axis) measurement `u` as input.
"""
@inline @fastmath function predict!(kf::IMUKalmanFilter, u::Float32)
    kf.x = Ad*kf.x + Bd*u
    kf.P = Ad*kf.P*Ad' + kf.Q
    nothing
end

"""
    correct!(kf, y)

Update step: correct state and covariance using accelerometer angle measurement `y`.
"""
@inline @fastmath function correct!(kf::IMUKalmanFilter, y::Float32)
    PCt = kf.P*Cd'
    Sp  = (Cd*PCt)[1,1] + kf.R[1,1]
    innovation = SA[y] - Cd*kf.x
    nis = innovation[1]^2 / Sp

    # Soft gate: for outliers, inflate R so they're down-weighted, not dropped.
    g = kf.innovation_threshold
    R = nis > g ? kf.R[1,1] * (nis / g) : kf.R[1,1]
    S = (Cd*PCt)[1,1] + R

    K = PCt / S
    kf.x = kf.x + K*innovation
    kf.P = symmetrize((I2 - K*Cd) * kf.P)
    nis
end

function symmetrize(P)
    m = (P[1,2] + P[2,1]) / 2
    SA[P[1,1] m; m P[2,2]]
end

"""
    update!(kf, u, y)

Combined predict + correct step.

- `u`: gyro measurement (angular velocity)
- `y`: accelerometer angle measurement
"""
function update!(kf::IMUKalmanFilter, u::Float32, y::Float32)
    predict!(kf, u)
    return correct!(kf, y)
end

"""
    angle(kf)

Get the estimated angle from the filter state.
"""
angle(kf::IMUKalmanFilter) = kf.x[1]

"""
    bias(kf)

Get the estimated gyro bias from the filter state.
"""
bias(kf::IMUKalmanFilter) = kf.x[2]

# IMU calibration constants (from original C++ code)
const GYRO_OFFSET = 128.1f0
const GYRO_SCALE = 131.0f0

"""
    angle, gyro_x, gyro_z = compute_angles(ax, ay, az, gx, gy, gz)

Apply calibration to raw IMU readings and compute
- `angle`: Tilt angle from accelerometer (radians)
- `gyro_x`: Calibrated gyro x (angular velocity) for tilt derivative control
- `gyro_z`: Calibrated gyro z (angular velocity) for turn derivative control

Applies calibration to gyro readings and computes angle from accelerometer.

Use with Kalman filter like this:
```julia
angle, gyro_x, gyro_z = compute_angles(ax, ay, az, gx, gy, gz)
update!(kf, gyro_x, angle)
```
That is, the gyro x reading is used as control input, and the angle computed from accelerometer as measurement.

Note that the original implementation used raw accelerometer measurements (at 16384 LSB/g), whereas here we
precorrect the accelerometer to read a Float32 in g's. Similarly we intepret the gyro measurements as 
precalculated degrees per second.


# Arguments
- `kf`: IMUKalmanFilter instance
- `ax, ay, az`: Accelerometer readings (float32, g)
- `gx, gy, gz`: Gyroscope readings (float32, deg/sec)
"""
function compute_angles(ax::Float32, ay::Float32, az::Float32,
                       gx::Float32, gy::Float32, gz::Float32)
    # Calculate angle from accelerometer (radians to degrees)
    angle = atan(Float32(ay), Float32(az)) * (180.0f0 / pi)

    # Apply gyro calibration offset and scale
    gyro_x = gx #(gx - GYRO_OFFSET) / GYRO_SCALE

    # Store z-axis gyro (for turn control)
    gyro_z = -gz # / GYRO_SCALE

    angle, gyro_x, gyro_z
end

