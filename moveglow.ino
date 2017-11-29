
frequency = 100; // Set reading frequency [Hz] - readings per second.

multiplier = 9.81; // Set multiplier to convert accelerometer data to acceleration m/s^2
acc_threshold = 0.2; // Set threshold to ignore small noise in acceleration
xAcc_Filtered = 0; // initialise filtered value to 0
acceleration_Filtered=0;
velocity_Filtered =0;


// coefficient to apply filtering, adjust these to give better result
alpha = 0.15;
beta = 0.95;
gamma = 0.9;


// Initialise variables for calculation
current_time = now;
t0 = now;
previous_time =0;
previous_velocity =0;
current_velocity = 0;
total_displacement = 0;


if (~exist('e', 'var'))
 e = engduino();
end



void Setup() {

//// Initialise accelerometer reading
for i=1:5
 newReading = e.getAccelerometer();
 gx = newReading(1);
 xAcc_Filtered = gx - ((1-alpha)*xAcc_Filtered + alpha*gx); // apply high pass filter to the accelerometer output
end
init_accx = (floor(xAcc_Filtered*100)*multiplier/100); // accelerometer data is multiplied to get 1g=10m/s^2

}



void Loop() {

current_time = (now - t0)*10e4; // Record the current time
newReading = e.getAccelerometer();
gx = newReading(1);
xAcc_Filtered = gx - ((1-alpha)*xAcc_Filtered + alpha*gx); // apply high pass filter to the accelerometer output
acceleration = (floor(xAcc_Filtered*100)*multiplier/100 - init_accx); // convert to acceleration from accelerometer

accFilt = (1-beta)*accFilt + beta*acceleration;
// ignore small value acceleration due to noise
if(accFilt>-acc_threshold&&accFilt<acc_threshold)
 accFilt = 0;
end

x=sym(acceleration_Filtered);

// Calculate velocity
current_velocity = previous_velocity + int(x, previous_time,
current_time);
// low pass filter to filter out noise from calculated velocity
velocity_Filtered = (1-gamma)*velocity_Filtered + gamma*current_velocity;
// Integrate velocity to displacement
displacement = int(current_velocity, previous_time, current_time);
total_displacement = total_displacement + displacement;
previous_velocity = velocity_Filtered;
previous_time = current_time;



pause(1/frequency);


}








