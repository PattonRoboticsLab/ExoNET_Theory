% ***********************************************************************
% Draw individual MARIONETs for the leg pose specified by phis
% ***********************************************************************

function h = drawExonetsLeg(p,phis)

%% Setup
fprintf('\n\n\n\n Drawing MARIONETs~~\n')
global EXONET BODY


%% Locations for the cartoon
Colors = [0.5 0.7 1; 0.1 1 0.2; 1 0.6 0.3]; % 3 distinct RGB color spaces
LW = 4; % lines width

hip = [0, 0]; % HIP position
knee = [BODY.Lengths(1)*sind(phis(1)), ... % KNEE position
        -(BODY.Lengths(1)*cosd(phis(1)))];
ankle = [knee(1) + BODY.Lengths(2)*sind(phis(1)-phis(2)); ... % ANKLE position
         knee(2) - BODY.Lengths(2)*cosd(phis(1)-phis(2))];

hipIndex = 1;
kneeIndex = EXONET.nParameters*EXONET.nElements+1;
hipKneeIndex = EXONET.nParameters*EXONET.nElements*2+1;


%% Loop thorough all MARIONETs
for element = 1:EXONET.nElements
    fprintf(' Hip element %d..',element);
    r = p(hipIndex+(element-1)*EXONET.nParameters+0);
    theta = p(hipIndex+(element-1)*EXONET.nParameters+1);
    L0 = p(hipIndex+(element-1)*EXONET.nParameters+2);
    rPos = [r*sind(theta) -r*cosd(theta)];     % R vector
    knee = [BODY.Lengths(1)*sind(phis(1)), ... % KNEE position
            -(BODY.Lengths(1)*cosd(phis(1)))];
    plot([rPos(1) knee(1)],[rPos(2) knee(2)],'Color',Colors(1,:),'Linewidth',LW);
    plot([hip(1) rPos(1)],[hip(2) rPos(2)],'Color',[0 0.2 0.9],'Linewidth',1);
end

for element = 1:EXONET.nElements
    fprintf(' Knee element %d..',element);
    r = p(kneeIndex+(element-1)*EXONET.nParameters+0);
    theta = p(kneeIndex+(element-1)*EXONET.nParameters+1);
    L0 = p(kneeIndex+(element-1)*EXONET.nParameters+2);
    knee = [BODY.Lengths(1)*sind(phis(1)), ...    % KNEE position
            -(BODY.Lengths(1)*cosd(phis(1)))];
    rPos = knee + [r*sind(theta) -r*cosd(theta)]; % R vector
    ankle = [knee(1) + BODY.Lengths(2)*sind(phis(1)-phis(2)), ... % ANKLE position
             knee(2) - BODY.Lengths(2)*cosd(phis(1)-phis(2))];
    plot([rPos(1) ankle(1)],[rPos(2) ankle(2)],'Color',Colors(2,:),'Linewidth',LW);
    plot([knee(1) rPos(1)],[knee(2) rPos(2)],'Color',[0 0.7 0],'Linewidth',1);
end

if EXONET.nJoints == 3
    for element = 1:EXONET.nElements
        fprintf(' 2-joints element %d..',element);
        r = p(hipKneeIndex+(element-1)*EXONET.nParameters+0);
        theta = p(hipKneeIndex+(element-1)*EXONET.nParameters+1);
        L0 = p(hipKneeIndex+(element-1)*EXONET.nParameters+2);
        rPos = [r*sind(theta) -r*cosd(theta)];                        % R vector
        ankle = [knee(1) + BODY.Lengths(2)*sind(phis(1)-phis(2)); ... % ANKLE position
                 knee(2) - BODY.Lengths(2)*cosd(phis(1)-phis(2))];
        plot([rPos(1) ankle(1)],[rPos(2) ankle(2)],'Color',Colors(3,:),'Linewidth',LW);
        plot([hip(1) rPos(1)],[hip(2) rPos(2)],'Color',[0.9 0.4 0],'Linewidth',1);
    end
end

fprintf('\n\n\n\n Done drawing~~\n')

end