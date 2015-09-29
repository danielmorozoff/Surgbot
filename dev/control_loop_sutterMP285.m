%%This is the main control function for operating the craniotomy
% Principle idea is to start at a point and then move in x^2+y^2 = r^2
% But in order to maintain thickness measure drill points should be relatively close to each other,
% but far enough away not to have liquid short the drill.


function control_loop_sutterMP285(sutter,radius, distance_btw_holes)

%     start point represents the center fo the circle
    [stepMult, currentVelocity, vScaleFactor]=getStatus(sutter);
    setOrigin(sutter);
    scaling_factor = 1;
   
    %Add a step_size that is proportional to the linead distance between
    %hole centers
    radial_step_size = calculate_radial_step(distance_btw_holes);
    for ang=0:radial_step_size:2*pi
%         Scale to the device
        xp=r*cos(ang);
        yp=r*sin(ang);
       
        x = scaling_factor*xp;
        y = scaling_factor*xp;
       
        z= 0 ; %This can be modififed in order to evaluate how to detect the surface of the bone
%         moveTo(sutter,[x y z]);
%         stepDrill();
       
    end
   

end
function stepDrill(z_stepSize)
   
    moveDeeper = drillEval();
    if (moveDeeper)
        stepDrill(z_stepSize);
    end
end
% This function handles the measuring of the signal.
function moveDeeper = drillEval()
% Turn probe pulse on.

% Record

% Analyze peak at proper frequency

    if (condition)
        moveDeeper = 1;
    else
        moveDeeper = 0;
    end

end
function circle(x,y,r)
    %x and y are the coordinates of the center of the circle
    %r is the radius of the circle
    %0.01 is the angle step, bigger values will draw the circle faster but
    %you might notice imperfections (not very smooth)
    ang=0:0.01:2*pi;
    xp=r*cos(ang);
    yp=r*sin(ang);
    plot(x+xp,y+yp);
end