function speed_control()
%%% Constants
r = 0.32;   % [m] Wheel radius
N = 50;      % Gear reduction
V_max = 48; % [V] Max voltage
i_cont = 100;   % [A] Max continuous current
i_max = 200;    % [A] Max current
Kv = 77;        % [RPM/V] Motor constant
Kt = 0.13;      % [Nm/A]  Motor constant
Ra = 0.010;     % [ohm] Coil resistance
i_nl = 13.5;    % [A] No load current, including gearbox internal torque
m = 250;        % [kg] Per-motor inertial mass

%%% Sim variables
dt = 0.001;     % [s] Loop step
v_fin = 1;      % [m/s] Desired speed
a_fin = 0.5;    % [m/s^2] Desired accel
t_total = 4;   % [s] Loop length
t_size = t_total/dt;    % Amount of loop iterations

%%% System state
f_load = 2000;   % [N] Our max drawbar load, divided by four
v_ref = 0;      % [m/s] Our current speed setpoint
rpm_act = 0;    % [RPM] Actual motor RPM
err = 0;   % [RPM] Error value
err_last = 0;   % [RPM] Last error value
err_int = 0;    % [RPM] Integrated error

%%% Long term storage
idx = 1;
t_ = dt:dt:t_total;
v_ref_ = zeros(1,t_size);
v_act_ = zeros(1,t_size);
u_ = zeros(1,t_size);
i_avail_ = zeros(1,t_size);
i_res_ = zeros(1,t_size);

%%% Sim loop
for t=t_
    % What speed and resistive torque are we working with right now?
    v_ref = get_speed(v_fin,a_fin,v_ref,dt);
    v_ref_(idx) = v_ref;
    rpm_ref = to_motor_rpm(v_ref,r,N);
    
    % Error and control loop
    err_last = err;
    err = rpm_ref - rpm_act;
    err_int = err_int + err;
    u = pid(err, err-err_last, err_int);
    if u > V_max
        u = V_max;
    elseif u < -V_max
        u = -V_max;
    end
    u_(idx) = u;
    
    % Simulation
    % Intermittent loading at 1 Hz
    if mod(floor(t),2)
        i_load = force_to_current(f_load,r,N,Kt);
    else
        i_load = 0;
    end
    i_res_(idx) = i_load+i_nl;
    
    % Basics
    i_avail = (u-rpm_act/Kv)/Ra;    % [A] Current through motor
    i_avail_(idx) = i_avail;
    i_accel = i_avail - i_nl - i_load; % [A] Current used for accel
    t_m = Kt * i_accel; % [N-m] Torque on motor
    force = t_m*N/r;    % [N] External force applied
    accel = force/m;    % [m/s^2] Vehicle acceleration
    accel_rpm = to_motor_rpm(accel,r,N);    % RPM increase per second
    rpm_act = rpm_act + dt*accel_rpm;       % Propagate RPM increase
    v_act = from_motor_rpm(rpm_act,r,N);    % Convenience
    v_act_(idx) = v_act;
    idx = idx + 1;
end

%%% Plot
close all;
figure(1);
subplot(3,1,1);
plot(t_,v_ref_,'b');
hold on;
plot(t_,v_act_,'r');
hold off;
legend('Ref','Act');
ylabel('m/s');
subplot(3,1,2);
plot(t_,u_);
ylabel('u');
subplot(3,1,3);
plot(t_,i_avail_,'b');
hold on;
plot(t_,i_res_,'r');
legend('Applied','Frict/Load');
ylabel('i');

end

%%% Subfunctions
function i = force_to_current(f,r,N,Kt)
    torque = f*r/N;
    i = torque/Kt;
end

function rpm = to_motor_rpm(v,r,N)
    omega_m = N*v/r;    % [rad/s] Angular motor speed
    rpm = omega_m * 60 / (2*pi);   % [RPM]
end

function vel = from_motor_rpm(rpm,r,N)
    vel = 2*pi*rpm*r/(60*N);    % [m/s] Vehicle speed
end

function speed = get_speed(v_ref, a_ref, v_cur, dt)
    accel = a_ref*sign(v_ref-v_cur);
    speed = v_cur + accel*dt;
    if accel > 0 && speed > v_ref 
        speed = v_ref;
    elseif accel < 0 && speed < v_ref
        speed = v_ref;
    end
end

function voltage = pid(e,e_d,e_i)
    k_p = 0.1;
    k_i = 0.05;
    k_d = 0;
    voltage = k_p*e + k_i*e_i + k_d*e_d;
end