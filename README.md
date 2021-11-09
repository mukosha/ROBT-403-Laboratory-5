# Mukhamedzhan Nurmukhamed 
# ROBT 403 Laboratory 5

# ROBT 403 Laboratory 5: Joint Control of 3-DOF RRR Manipulator

### Note: The aim of this project was to create the model and provide the joint control of RRR Manipulator. This task was accomplished in MatLab via the use of the external toolbox called “Robotics Toolbox from Peter Corke”: [Peter Corke's website](https://petercorke.com/)



## Requirements
The project has no specific requirements except the software.


## The lab 5 task achievements

### I. The first step was to conceptualize and affix the frames to each corresponding link:
![image](https://user-images.githubusercontent.com/47817099/140949407-b2f89387-504e-424b-a46c-c4118fd161db.png)

### II. The next step was to find the DH parameters of the RRR-Manipulator:

| joint i | alpha | a | d | Ө |
| -- | -- | -- | -- | -- |
| 1 | 90 deg | 0 | 1 | Ө1 |
| 2 | 0 deg | 1 | 0 | Ө2 |
| 3 | 0 deg | 1 | 0 | Ө3 |

### III. Next, by considering each of the links of the RRR-Manipulator as a cylinder (homogeneous density with radius of cross-section r = 0.05 m), we are able to calculate the inertia matrix:
      
         [ 0 0 0 ]         [ 0.0125  0       0      ]              [ 0.0125  0       0      ] 
    I1 = [ 0 0 0 ]    I2 = [ 0       0.8396  0      ]         I2 = [ 0       0.8396  0      ]
         [ 0 0 0 ]         [ 0       0       0.8396 ]              [ 0       0       0.8396 ]
     
### IV. Now, we are able to model the RRR-Manipulator in MatLab:
```script
L(1) = Revolute('d', 1, 'a', 0, 'alpha', pi/2, ...
    'I', [0, 0, 0], ...
    'r', [0, 0, 0.5], ...
    'm', 0, ...
    'Jm', 1e-4, ...
    'G', 500, ...
    'B', 10e-4, ...
    'Tc', 10e-4, ...
    'qlim', [-180 180]*rad_to_deg );

L(2) = Revolute('d', 0, 'a', 1, 'alpha', 0, ...
    'I', [0.0125, 0.83958, 0.83958], ...
    'r', [0.5, 0, 0], ...
    'm', 10, ...
    'Jm', 1e-4, ...
    'G', 500, ...
    'B', 10e-4, ...
    'Tc', 10e-4, ...
    'qlim', [-90 90]*rad_to_deg );

L(3) = Revolute('d', 0, 'a', 1, 'alpha', 0,  ...
    'I', [0.0125, 0.83958, 0.83958], ...
    'r', [1.5, 0, 0], ...
    'm', 10, ...
    'Jm', 1e-4, ...
    'G', 500, ...
    'B', 10e-4, ...
    'Tc', 10e-4, ...
    'qlim', [-90 90]*rad_to_deg );
```

### V.  Hence, we are able to obtain the plot of the manipulator:
{1}
```script
RRR_Manipulator.plot([0, 0, 0]) 
```
![image](https://user-images.githubusercontent.com/47817099/140951166-5a6bb616-c4f5-4a1b-8187-ea42d897ce31.png)

{2} 
```script
RRR_Manipulator.teach()
```
![image](https://user-images.githubusercontent.com/47817099/140951298-be2fbb79-ef1c-465b-a3ca-bbd11ff0df12.png)



### VI. In this step, we find the torques that are associated with the joints that keep the RRR-Manipulator at its corresponding home position and hold position:
#### {1} Home position:
```script
figure(1)
RRR_Manipulator.plot(position_HOME)
RRR_Manipulator.teach()
```
![image](https://user-images.githubusercontent.com/47817099/140951825-73b12b17-284a-4432-a1ca-759868e09ea1.png)

```script
%Home position of the RRR-Manipulator
Jacobian_1 = RRR_Manipulator.jacob0(position_HOME);
gravitational_load_1 = RRR_Manipulator.gravload([0, 0, 1]);
Torque_1 = Jacobian_1.*gravitational_load_1
```
![image](https://user-images.githubusercontent.com/47817099/140951957-ac572af7-70dd-49c5-8cdb-2eaf256dbf27.png)

#### {2} Hold position:
```script
figure(2)
RRR_Manipulator.plot(position_HOLD)
RRR_Manipulator.teach()
```
![image](https://user-images.githubusercontent.com/47817099/140951892-2abb4a3a-f9a3-4b09-b1e4-66dcf140194b.png)

```script
%Hold position of the RRR-Manipulator
Jacobian_2 = RRR_Manipulator.jacob0(position_HOLD);
gravitational_load_2 = RRR_Manipulator.gravload([0, 0, 1]);
Torque_2 = Jacobian_2.*gravitational_load_2
```
![image](https://user-images.githubusercontent.com/47817099/140951988-44ce6d93-cb79-46e1-8162-464c7e6cb14e.png)

##*** The results suggest that the numerical value of the hold position torque is bigger than that of the home position. Hence, we can conclude that the MatLab model and calculations were correct.


## The full MATLAB script:

```script
% Mukhamedzhan Nurmukhamed 
%ROBT 403 Laboratory 5: Joint Control of 3-DOF RRR Manipulator

clear;
rad_to_deg = pi/180;

L(1) = Revolute('d', 1, 'a', 0, 'alpha', pi/2, ...
    'I', [0, 0, 0], ...
    'r', [0, 0, 0.5], ...
    'm', 0, ...
    'Jm', 1e-4, ...
    'G', 500, ...
    'B', 10e-4, ...
    'Tc', 10e-4, ...
    'qlim', [-180 180]*rad_to_deg );

L(2) = Revolute('d', 0, 'a', 1, 'alpha', 0, ...
    'I', [0.0125, 0.83958, 0.83958], ...
    'r', [0.5, 0, 0], ...
    'm', 10, ...
    'Jm', 1e-4, ...
    'G', 500, ...
    'B', 10e-4, ...
    'Tc', 10e-4, ...
    'qlim', [-90 90]*rad_to_deg );

L(3) = Revolute('d', 0, 'a', 1, 'alpha', 0,  ...
    'I', [0.0125, 0.83958, 0.83958], ...
    'r', [1.5, 0, 0], ...
    'm', 10, ...
    'Jm', 1e-4, ...
    'G', 500, ...
    'B', 10e-4, ...
    'Tc', 10e-4, ...
    'qlim', [-90 90]*rad_to_deg );

RRR_Manipulator = SerialLink(L, 'name', 'RRR-Manipulator (3 DOF)');

position_HOME = [0 pi/2 0];
position_HOLD = [0 0 0]; 
clear L

figure(1)
RRR_Manipulator.plot(position_HOME)
RRR_Manipulator.teach()

%Home position of the RRR-Manipulator
Jacobian_1 = RRR_Manipulator.jacob0(position_HOME);
gravitational_load_1 = RRR_Manipulator.gravload([0, 0, 1]);
Torque_1 = Jacobian_1.*gravitational_load_1

figure(2)
RRR_Manipulator.plot(position_HOLD)
RRR_Manipulator.teach()

%Hold position of the RRR-Manipulator
Jacobian_2 = RRR_Manipulator.jacob0(position_HOLD);
gravitational_load_2 = RRR_Manipulator.gravload([0, 0, 1]);
Torque_2 = Jacobian_2.*gravitational_load_2

```


## Thank you for your attention!
