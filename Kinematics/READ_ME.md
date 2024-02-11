# Kinematics

# Forward Kinematics

Generally, the Forward Kinematics (FPK) is given by the joint values (theta values) to calculate the pose includes position and orientation of end-effector (Baxter's gripper is end-effector)

**Given:** $$(\theta_0,\theta_1,\theta_2,\theta_3,\theta_4,\theta_5,\theta_6)$$

**Calculate**: $$\left[ _{7}^{0}T \right] and \left[ _{G}^{W}T \right]$$

**Where** : {G} is the left-arm end-effector(gripper) frame and {W} is the World fixed reference frame on the floor.

For example, I set all $`\theta`$ in forward_kinematic.py are 0.
**First**
I import 'numpy' library
```python
import numpy as np
```

**Kinematic code (1)**


```python
# Link lengths
L0 = 270.35
L1 = 69.00
L2 = 364.35
L3 = 69.00
L4 = 374.29
L5 = 10.00
L6 = 368.30
```

```python
# Joint angles (all set to zero)
theta1 = np.pi/18
theta2 = np.pi/9
theta3 = np.pi/6
theta4 = (2*np.pi)/9
theta5 = (5*np.pi)/18
theta6 = np.pi/3
theta7 = (np.pi*7)/18
```





