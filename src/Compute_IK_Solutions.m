function joint_angles_deg = compute_IK_solutions(Height_of_person, x_target, y_target)

    foot_l = 0.152*Height_of_person*0.8;
    leg_l = (0.285-0.039)*Height_of_person;
    thigh_l = (0.48-0.285)*Height_of_person;
    thn_l = (0.818-0.48)*Height_of_person;
    uparm_l = 0.186*Height_of_person;
    lowarm_l = (0.145+0.108)*Height_of_person;
    vars = [foot_l leg_l thigh_l thn_l uparm_l lowarm_l x_target y_target];
% Two endpoint initial guesses (in degrees)
% [ankle, knee, hip, spine, shoulder, elbow]
    guess1 = [180, -140, 160, -135, -170, 28];  % deep squat with bent elbow
    guess2 = [180, -80, 0, -105, -90, 10];  % locked knees with straight arm
%radians
    guess1 = guess1 * pi/180;
    guess2 = guess2 * pi/180;
    num_guesses = 20;
    all_guesses = zeros(num_guesses, 6);
for i = 1:num_guesses
        t = (i-1)/(num_guesses-1);  % Parameter from 0 to 1
        all_guesses(i, 1) = guess1(1);  % Ankle stays constant
        all_guesses(i, 2:6) = (1-t)*guess1(2:6) + t*guess2(2:6); %this
%part of the code is able to weight how much each part of the guess
%the specific frame has.
%example:
%i = 1 is 100% guess 1 and i = 20 is 100% guess2
end
    options = optimset('Display', 'notify');
% Solve for all guesses
    solutions = cell(num_guesses, 1);
for i = 1:num_guesses
        guess = all_guesses(i, :);
%this is the powerhouse of this function. It changes the Linkage
%function to a function handle with a single variable of a which is
%a vector of 6 angles. Since it is a numerical function it needs
%initial guesses. Also, I only cared about the angles in this
%situation, therefore the ~s.
        [angles, ~, ~] = fsolve(@(angles) Linkage(angles, vars), guess, options);
        solutions{i} = angles;
end
% Create matrix of all 20 joint angle solutions
    joint_angles_rad = zeros(num_guesses, 6);
for i = 1:num_guesses
        joint_angles_rad(i, :) = solutions{i};
end
% Convert to degrees for output
    joint_angles_deg = joint_angles_rad * 180/pi;
end
function eqns = Linkage(angles, vars)
% Extract parameters
    foot_l = vars(1);
    leg_l = vars(2);
    thigh_l = vars(3);
    thn_l = vars(4);
    uparm_l = vars(5);
    lowarm_l = vars(6);
    x_target = vars(7);
    y_target = vars(8);
% Extract angles
    a1 = angles(1);
    a2 = angles(2);
    a3 = angles(3);
    a4 = angles(4);
    a5 = angles(5);
    a6 = angles(6);
% Forward kinematics: compute end effector position
    x_end = foot_l*cos(a1) + leg_l*cos(a1+a2) + thigh_l*cos(a1+a2+a3) + ...
            thn_l*cos(a1+a2+a3+a4) + uparm_l*cos(a1+a2+a3+a4+a5) + ...
            lowarm_l*cos(a1+a2+a3+a4+a5+a6);
    y_end = foot_l*sin(a1) + leg_l*sin(a1+a2) + thigh_l*sin(a1+a2+a3) + ...
            thn_l*sin(a1+a2+a3+a4) + uparm_l*sin(a1+a2+a3+a4+a5) + ...
            lowarm_l*sin(a1+a2+a3+a4+a5+a6);
% Equations to solve (difference from target)
    eqns(1) = x_end - x_target;
    eqns(2) = y_end - y_target;
end