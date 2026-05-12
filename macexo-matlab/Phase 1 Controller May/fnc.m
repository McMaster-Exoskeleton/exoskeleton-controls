function thetaDoublePrime = fnc(theta, thetaPrime, t_command, t_gravity_load, D_friction, I_link, I_joint)

thetaDoublePrime = ((t_command - t_gravity_load - D_friction*thetaPrime)) / (I_link + I_joint);