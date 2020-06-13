function AP_send(gyro, attitude, accel, velocity, position, time)
global u
persistent past_time
if isempty(past_time)
    past_time = -1;
end
if past_time == time
    warning('Repeat time 2');
    return
end
fprintf('DT = %0.8f\n',time-past_time)
past_time = time;

% build structure representing the JSON string to be sent
JSON.timestamp = time;
JSON.imu.gyro = gyro;
JSON.imu.accel_body = accel;
JSON.position = position;
JSON.attitude = attitude;
JSON.velocity = velocity;

% Report to AP
pnet(u,'printf',sprintf('\n%s\n',jsonencode(JSON)));
pnet(u,'writepacket');

end