/*   Moktarino's Fancy Dan Dancin' Belt
 *   
 *   This project entails making a motion-reactive display in the form of a belt.
 *   The motion data is provided by a MPU6050 IMU (GY-521 board in my case) and 
 *   displayed on a neopixel array.  The current effect, with proper positioning, will
 *   allow a dancer to roll a colored gradient around their waist by hip motion alone.
 *   
 *   I'm looking forward to adding more effects, another IMU, and other dimensions
 *   to make this another way a dancer can express complex movements visually.
 *   
 *   Much of this code (i2c read/write functions, complementary filter) I cribbed from 
 *   Kristian Lauszus, whose work with the MPU6050 can be found here: https://github.com/TKJElectronics
 *   
 *   The formula for calculating the location of the peak pixel came courtesy of
 *   redditor /u/benargee.  Thanks!
 *   
 *   Please please please contact me with improvement suggestions or ideas for effects, 
 *   especially with tips on how to implement them in code!
 *   
 *   Happy dancing!
 *   
 *   Moktarino@gmail.com
 *     
 */
