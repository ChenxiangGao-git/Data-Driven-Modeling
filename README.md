# Data_Driven_Modeling

This program demonstrates the application of the Data-Driven Modeling Method for system modeling.

The underlying principles are thoroughly explained in Section II of the article "Hybrid Data-Physics-Driven Modeling Method for Real-Time Simulation of Cascaded Power Electronics Systems" by C. Gao et al.

Cases of CHB-DAB, Boost, and 2-level VSC are presented, including both the trapezoidal method and the backward Euler method.

1、The script Generate_T_Matrix.m constructs the transmission matrix T using an offline simulation model, with results stored in T_Matrix.mat.

2、The accuracy is validated in Test_T_Matrix.m by comparing Simulink simulation results with those derived from the transmission matrices.
