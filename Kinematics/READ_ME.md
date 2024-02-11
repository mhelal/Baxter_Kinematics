# Kinematics

# Forward Kinematics

Generally, the Forward Kinematics (FPK) is given by the joint values to calculate the pose includes position and orientation of end-effector (Baxter's gripper is end-effector)

\left[ _{i-1}^{i}T \right] = \begin{bmatrix}
c\theta_i & -s\theta_i & 0 & a_{i-1} \\
s\theta_i c\alpha_{i-1} & c\theta_i c\alpha_{i-1} & -s\alpha_{i-1} & -d_i s\alpha_{i-1} \\
s\theta_i s\alpha_{i-1} & c\theta_i s\alpha_{i-1} & c\alpha_{i-1} & d_i c\alpha_{i-1} \\
0 & 0 & 0 & 1
\end{bmatrix} = \left[ \begin{array}{cc}
_{i-1}^{i}R & _{i-1}^{i}P \\
0 & 1
\end{array} \right]



