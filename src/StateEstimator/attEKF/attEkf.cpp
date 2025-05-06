#include "attEkf.h"

AttStateEstimator::AttStateEstimator() {
    // Initialize Error Covariance

    P.Fill(0.0f);
    for(uint8_t idx : AttKFInds::quat) {
        P(idx, idx) = 1e-8;
    }
    for(uint8_t idx : AttKFInds::gyroBias) {
        P(idx, idx) = powf(icm20948_const.gyro_VRW, 2.0f);
    }
    for(uint8_t idx: AttKFInds::accelBias) {
        P(idx, idx) = powf(icm20948_const.accelXY_VRW, 2.0f);
    }
    for(uint8_t idx : AttKFInds::magBias) {
        // P(idx, idx) = icm20948_const.magXYZ_var;
        P(idx, idx) = powf(0.1f, 2);
    }
    
    // P(AttKFInds::accelBias[0], AttKFInds::accelBias[0]) = icm20948_const.accelXY_var;
    // P(AttKFInds::accelBias[1], AttKFInds::accelBias[1]) = icm20948_const.accelXY_var;
    // P(AttKFInds::accelBias[2], AttKFInds::accelBias[2]) = icm20948_const.accelZ_var;
    // P(AttKFInds::accelBias[0], AttKFInds::accelBias[0]) = 0.01;
    // P(AttKFInds::accelBias[1], AttKFInds::accelBias[1]) = 0.01;
    // P(AttKFInds::accelBias[2], AttKFInds::accelBias[2]) = 0.01;
    
    P_min = P;

    // Initialize Process Noise
    Q.Fill(0.0f);
    for(uint8_t idx : AttKFInds::quat) { 
        Q(idx, idx) = 1e-8;
    }
    for(uint8_t idx : AttKFInds::gyroBias) {
        Q(idx, idx) = powf(0.01f,2);
    }
    for(uint8_t idx : AttKFInds::accelBias) {
        Q(idx, idx) = powf(0.01f,2);
    }
    for(uint8_t idx : AttKFInds::magBias) {
        Q(idx, idx) = powf(0.1f, 2);
    }

    Serial.println("<----- Process Noise ----->");
    for (int i = 0; i < Q.Rows; i++) {
        for (int j = 0; j < Q.Cols; j++) {
            Serial.print(String(Q(i,j)) + "\t");
        }
        Serial.println("");
    }

    Serial.println("<----- Initial Error Cov ----->");
    for (int i = 0; i < P.Rows; i++) {
        for (int j = 0; j < P.Cols; j++) {
            Serial.print(String(P(i,j)) + "\t");
        }
        Serial.println("");
    }
}

void AttStateEstimator::init(BLA::Matrix<13,1> x_0, float dt) {
    this->dt = dt;

    this->x     = x_0;
    this->x_min = x_0;
}

BLA::Matrix<13,1> AttStateEstimator::onLoop(Context &ctx) {
    // Read data from sensors and convert values
    
    float gyrX = ctx.mag.getData().gyrX;
    float gyrY = ctx.mag.getData().gyrY;
    float gyrZ = ctx.mag.getData().gyrZ;

    float aclX = ctx.mag.getData().accelX / g;
    float aclY = ctx.mag.getData().accelY / g;
    float aclZ = ctx.mag.getData().accelZ / g;

    float magX = ctx.mag.getData().magX;
    float magY = ctx.mag.getData().magY;
    float magZ = ctx.mag.getData().magZ;

    BLA::Matrix<3,1> u = {gyrX, gyrY, gyrZ};   // [rads]
    BLA::Matrix<3,1> a_b = {aclX, aclY, aclZ}; // [g]
    BLA::Matrix<3,1> m_b = {magX, magY, magZ}; // [uT]

    // Update u_prev if first iteration
    if(!hasPassedGo) {
        u_prev = u;

        hasPassedGo = true;
    }

    x_min = x + predictionFunction(x, u) * dt;
    // x_min = propRK4(u);

    // Measurement Jacobian Matrix
    BLA::Matrix<13,13> F = predictionJacobian(u);

    // Discretize Measurement
    BLA::Matrix<13,13> phi = I_13 + F * dt;

    // Predict Error Covariance
    P_min = phi * F * BLA::MatrixTranspose<BLA::Matrix<13,13>>(phi) + Q;

    // ===== IF ON PAD ===== TODO
    applyGravUpdate(x_min, a_b);

    // ===== ALWAYS =====
    // applyMagUpdate(x, m_b);
    // APPLY MAG UPDATE

    // === Ensure P is Symmetric ===
    P = (P + BLA::MatrixTranspose<BLA::Matrix<13,13>>(P)) * 0.5f;

    // Update previous gyro reading
    u_prev = u;

    ctx.q_w = x(AttKFInds::q_w);
    ctx.q_x = x(AttKFInds::q_x);
    ctx.q_y = x(AttKFInds::q_y);
    ctx.q_z = x(AttKFInds::q_z);

    ctx.quatState = x;
    ctx.P = P;

    return x;
}

BLA::Matrix<13,1> AttStateEstimator::propRK4(BLA::Matrix<3,1> u) {

    BLA::Matrix<3,1> u_k1  = u_prev;
    BLA::Matrix<3,1> u_k   = u;
    BLA::Matrix<3,1> u_k12 = 0.5f * (u_k1 + u_k);

    BLA::Matrix<13,1> k1 = dt * predictionFunction(x, u_k1);
    BLA::Matrix<13,1> k2 = dt * predictionFunction(x + k1*0.5f, u_k12);
    BLA::Matrix<13,1> k3 = dt * predictionFunction(x + k2*0.5f, u_k12);
    BLA::Matrix<13,1> k4 = dt * predictionFunction(x + k3, u_k);

    x_min = x + (k1*(1.0f/6.0f) + k2*(1.0f/3.0f) + k3*(1.0f/3.0f) + k4*(1.0f/6.0f));

    // Force Normalize Unit Quaternion
    BLA::Matrix<4,1> quat = {
        x_min(AttKFInds::q_w),
        x_min(AttKFInds::q_x),
        x_min(AttKFInds::q_y),
        x_min(AttKFInds::q_z)
    };

    quat = quat / BLA::Norm(quat);

    x_min(AttKFInds::q_w) = quat(0);
    x_min(AttKFInds::q_x) = quat(1);
    x_min(AttKFInds::q_y) = quat(2);
    x_min(AttKFInds::q_z) = quat(3);

    return x_min;
}

BLA::Matrix<13,1> AttStateEstimator::predictionFunction(BLA::Matrix<13,1> x, BLA::Matrix<3,1> u) {
    float p = u(0) - x(AttKFInds::gb_x);
    float q = u(1) - x(AttKFInds::gb_y);
    float r = u(2) - x(AttKFInds::gb_z);

    BLA::Matrix<4,3> quatMat = {
        -x(1), -x(2), -x(3),
         x(0), -x(3),  x(2),
         x(3),  x(0), -x(1), 
        -x(2),  x(1),  x(0), 
    };
    
    BLA::Matrix<3,1> gyr = {p, q, r};

    BLA::Matrix<4, 1> f_q = (quatMat * gyr) * 0.5f;

    // Assume constant bias
    BLA::Matrix<13,1> f = {
        f_q(0), f_q(1), f_q(2), f_q(3), 0, 0, 0, 0, 0, 0, 0, 0, 0
    };

    return f;
}

BLA::Matrix<13,13> AttStateEstimator::predictionJacobian(BLA::Matrix<3,1> u) {
    float gbx = x(AttKFInds::gb_x);
    float gby = x(AttKFInds::gb_y);
    float gbz = x(AttKFInds::gb_z);
    
    float p = u(0) - gbx;
    float q = u(1) - gby;
    float r = u(2) - gbz;

    float qw = x(AttKFInds::q_w);
    float qx = x(AttKFInds::q_x);
    float qy = x(AttKFInds::q_y);
    float qz = x(AttKFInds::q_z);

    BLA::Matrix<13,13> F = {
        0, gbx/2 - p/2, gby/2 - q/2, gbz/2 - r/2,  qx/2,  qy/2,  qz/2, 0, 0, 0, 0, 0, 0,
        p/2 - gbx/2,           0, r/2 - gbz/2, gby/2 - q/2, -qw/2,  qz/2, -qy/2, 0, 0, 0, 0, 0, 0,
        q/2 - gby/2, gbz/2 - r/2,           0, p/2 - gbx/2, -qz/2, -qw/2,  qx/2, 0, 0, 0, 0, 0, 0,
        r/2 - gbz/2, q/2 - gby/2, gbx/2 - p/2,           0,  qy/2, -qx/2, -qw/2, 0, 0, 0, 0, 0, 0,
        0,           0,           0,           0,     0,     0,     0, 0, 0, 0, 0, 0, 0,
        0,           0,           0,           0,     0,     0,     0, 0, 0, 0, 0, 0, 0,
        0,           0,           0,           0,     0,     0,     0, 0, 0, 0, 0, 0, 0,
        0,           0,           0,           0,     0,     0,     0, 0, 0, 0, 0, 0, 0,
        0,           0,           0,           0,     0,     0,     0, 0, 0, 0, 0, 0, 0,
        0,           0,           0,           0,     0,     0,     0, 0, 0, 0, 0, 0, 0,
        0,           0,           0,           0,     0,     0,     0, 0, 0, 0, 0, 0, 0,
        0,           0,           0,           0,     0,     0,     0, 0, 0, 0, 0, 0, 0,
        0,           0,           0,           0,     0,     0,     0, 0, 0, 0, 0, 0, 0,
    };

    return F;
}

void AttStateEstimator::applyGravUpdate(BLA::Matrix<13,1> &x_in, BLA::Matrix<3,1> a_b) {
    BLA::Matrix<3,1> G_NED = {0, 0, -1};

    BLA::Matrix<4,1> q = {
        x_in(AttKFInds::q_w),
        x_in(AttKFInds::q_x),
        x_in(AttKFInds::q_y),
        x_in(AttKFInds::q_z)
    };

    BLA::Matrix<3,3> R_TB = quat2rot(q);

    BLA::Matrix<3,1> bias = {
        x_in(AttKFInds::ab_x),
        x_in(AttKFInds::ab_y),
        x_in(AttKFInds::ab_z)
    };
    
    BLA::Matrix<3,1> h_grav = BLA::MatrixTranspose<BLA::Matrix<3,3>>(R_TB) * G_NED + bias;

    BLA::Matrix<3,1> z_grav = a_b - bias;

    float qw = x_in(AttKFInds::q_w);
    float qx = x_in(AttKFInds::q_x);
    float qy = x_in(AttKFInds::q_y);
    float qz = x_in(AttKFInds::q_z);

    float abx = x_in(AttKFInds::ab_x);
    float aby = x_in(AttKFInds::ab_y);
    float abz = x_in(AttKFInds::ab_z);

    BLA::Matrix<3, 13> H_grav = {
         2*qy, -2*qz,  2*qw, -2*qx, 0, 0, 0, 1, 0, 0, 0, 0, 0,
        -2*qx, -2*qw, -2*qz, -2*qy, 0, 0, 0, 0, 1, 0, 0, 0, 0,
        -4*qw,  0,       0,      -4*qz, 0, 0, 0, 0, 0, 1, 0, 0, 0,
    };

    BLA::Matrix<3,3> S = H_grav * P_min * BLA::MatrixTranspose<BLA::Matrix<3,13>>(H_grav) + R_grav;

    BLA::Matrix<13,3> K = P_min * BLA::MatrixTranspose<BLA::Matrix<3,13>>(H_grav) * BLA::Inverse(S);

    BLA::Matrix<3,1> y = z_grav - h_grav;

    x = x_in + K * (z_grav - h_grav);

    // Normalize quaternion
    BLA::Matrix<4,1> quat = {
        x(AttKFInds::q_w),
        x(AttKFInds::q_x),
        x(AttKFInds::q_y),
        x(AttKFInds::q_z)
    };

    quat = quat / BLA::Norm(quat);
    x(AttKFInds::q_w) = quat(0);
    x(AttKFInds::q_x) = quat(1);
    x(AttKFInds::q_y) = quat(2);
    x(AttKFInds::q_z) = quat(3);

    P = (I_13 - K * H_grav) * P_min;
}

void AttStateEstimator::applyMagUpdate(BLA::Matrix<13,1> &x_in, BLA::Matrix<3,1> m_b) {
    BLA::Matrix<3,1> B_NED = {19.9583f, -4.8770f, 47.0710f}; // [uT]

    float B_N = B_NED(0);
    float B_E = B_NED(1);
    float B_D = B_NED(2);

    BLA::Matrix<4,1> q = {
        x_in(AttKFInds::q_w),
        x_in(AttKFInds::q_x),
        x_in(AttKFInds::q_y),
        x_in(AttKFInds::q_z)
    };

    BLA::Matrix<3,3> R_TB = quat2rot(q);

    BLA::Matrix<3,1> bias = {
        x_in(AttKFInds::mb_x),
        x_in(AttKFInds::mb_y),
        x_in(AttKFInds::mb_z)
    };

    float qw = x_in(AttKFInds::q_w);
    float qx = x_in(AttKFInds::q_x);
    float qy = x_in(AttKFInds::q_y);
    float qz = x_in(AttKFInds::q_z);

    float mbx = x_in(AttKFInds::mb_x);
    float mby = x_in(AttKFInds::mb_y);
    float mbz = x_in(AttKFInds::mb_z);

    BLA::Matrix<3,1> h_mag = BLA::MatrixTranspose<BLA::Matrix<3,3>>(R_TB) * B_NED + bias;
    BLA::Matrix<3,1> z_mag = m_b - bias;

    BLA::Matrix<3,13> H_mag = {
        2*B_E*qz - 2*B_D*qy,                       
        2*B_D*qz + 2*B_E*qy,                         
        2*B_E*qx - 2*B_D*qw - 4*B_N*qy,              
        2*B_D*qx + 2*B_E*qw - 4*B_N*qz,           
        0, 0, 0, 0, 0, 0,
        1, 0, 0,                                      
    
        2*B_D*qx - 2*B_N*qz,                      
        2*B_D*qw - 4*B_E*qx + 2*B_N*qy,             
        2*B_D*qz + 2*B_N*qx,                          
        2*B_D*qy - 4*B_E*qz - 2*B_N*qw,               
        0, 0, 0, 0, 0, 0,
        0, 1, 0,                                     
    
        2*B_N*qy - 2*B_E*qx,                         
        2*B_N*qz - 2*B_E*qw - 4*B_D*qx,               
        2*B_E*qz - 4*B_D*qy + 2*B_N*qw,              
        2*B_E*qy + 2*B_N*qx,                        
        0, 0, 0, 0, 0, 0,
        0, 0, 1                                     
    };

    BLA::Matrix<3,3> S = H_mag * P_min * BLA::MatrixTranspose<BLA::Matrix<3,13>>(H_mag) + R_mag;

    BLA::Matrix<13,3> K = P_min * BLA::MatrixTranspose<BLA::Matrix<3,13>>(H_mag) * BLA::Inverse(S);

    x = x_in + K * (z_mag - h_mag);

    //Normalize quaternion
    BLA::Matrix<4,1> quat = {
        x(AttKFInds::q_w),
        x(AttKFInds::q_x),
        x(AttKFInds::q_y),
        x(AttKFInds::q_z)
    };
    quat = quat / BLA::Norm(quat);
    x(AttKFInds::q_w) = quat(0);
    x(AttKFInds::q_x) = quat(1);
    x(AttKFInds::q_y) = quat(2);
    x(AttKFInds::q_z) = quat(3);

    P = (I_13 - K * H_mag) * P_min;
}

BLA::Matrix<3,3> quat2rot(const BLA::Matrix<4,1> &q) {

    float q0 = q(0);
    float q1 = q(1);
    float q2 = q(2);
    float q3 = q(3);

    BLA::Matrix<3,3> rotm = {
        2*(q0*q0 + q1*q1)-1, 2*(q1*q2 - q0*q3), 2*(q1*q3 + q0*q2),
        2*(q1*q2 + q0*q3), 2*(q0*q0 + q2*q2)-1, 2*(q2*q3 - q0*q1),
        2*(q1*q3 - q0*q2), 2*(q2*q3 + q0*q1), 2*(q0*q0 + q3*q3)-1,
    };

    return rotm;

}