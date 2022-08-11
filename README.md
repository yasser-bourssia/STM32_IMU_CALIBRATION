This repository contains some of my attempts to calibrate an accelerometer on an STM32 Device (L4R5QII6).

It contains the initiation and the configuration of an accelerometer (LSM303D), and the calibration of the said accelerometer.

It also contains a sort of correction, that counter-acts the errors on the measures that come from the installation of the accelerometer on the vehicule.

It basically uses a rotation matrix to rotate the vectors back to an original 'horizontal' position even if the accelerometer isn't horizontally installed.

The matrix used is as follows:

![alt text](https://media.discordapp.net/attachments/793105657145720862/1007337335773474816/unknown.png)

Only the two first matrices are used; to use these formulaes, we need to first calculate two angles (roll and pitch), worth noting that we cannot calculate the third angle (yaw) from an accelerometer so only two angles were used in this application.

The computation of the angles is also included in the main.c file.

