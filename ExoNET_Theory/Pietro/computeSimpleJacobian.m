    % Compute Jacobian using the current arm configuration
    % This is a simplified approach using the arm vectors
function J = computeSimpleJacobian(p_shoulder, p_elbow, p_wrist, Bod, , robot, Exo, q)
    
    %% Arm vectors from shoulder, elbow and wrist position
      v_upper = p_elbow - p_shoulder;    % Upper arm vector
    v_forearm = p_wrist - p_elbow;       % Forearm vector
      v_total = p_wrist - p_shoulder;    % wrist to shoulder vector
    
    %% Check for problematic configurations and add small perturbations
    % min_length = 0.02; % Minimum segment length to avoid singularities
    % 
    % if norm(v_upper) < min_length
    %     v_upper = v_upper + [min_length; 0; 0];
    % end
    % if norm(v_forearm) < min_length
    %     v_forearm = v_forearm + [0; min_length; 0];
    % end
    % if norm(v_total) < min_length
    %     v_total = v_total + [0; 0; min_length];
    % end
    
    %% Check for fully extended arm (major singularity) and add perturbation
    % arm_extension = norm(v_total) / (norm(v_upper) + norm(v_forearm));
    % 
    % if arm_extension > 0.95  % Nearly fully extended
    %     % Add small perturbation to break singularity
    %     perturbation = 0.08 * [cos(rand*2*pi); sin(rand*2*pi); 0];
    %     v_forearm = v_forearm + perturbation;
    %     v_total = p_wrist + perturbation - p_shoulder;
    % end
    
    %% Check for arm folded back (elbow singularity)
    % dot_product = dot(v_upper, v_forearm) / (norm(v_upper) * norm(v_forearm));
    % 
    % if abs(dot_product) > 0.95  % Nearly aligned or anti-aligned
    %     % Add perpendicular perturbation
    %     perp_dir = cross(v_upper, [0; 0; 1]);
    %     if norm(perp_dir) < 0.01
    %         perp_dir = cross(v_upper, [0; 1; 0]);
    %     end
    %     perp_dir = perp_dir / norm(perp_dir);
    %     v_forearm = v_forearm + 0.02 * perp_dir;
    %     v_total = p_wrist + 0.02 * perp_dir - p_shoulder;
    % end
    % 
    % % Normalize vectors
    % v_upper_norm = v_upper / norm(v_upper);
    % v_forearm_norm = v_forearm / norm(v_forearm);
    % v_total_norm = v_total / norm(v_total);
    
    %% Create orthogonal vectors for each joint
    for j=1:length(p_elbow)
        q = computeJointAngles3D(p_shoulder,p_elbow,p_wrist);

        delta = 1e-6; J = zeros(3,length(q));
        for i=1:length(q)

            dq= zeros(size(q));
            dq(i)=delt;
            p1 = forwardKinematics(q);
            p2 = forwardKinematics(q+dq);
            J(:,i)= (p2-p1)/delta;

        end
    end

    %% Joint 1: Shoulder abduction (rotation about z-axis)
    % z_axis = [0; 0; 1];
    % J1 = cross(z_axis, v_total);
    % 
    % %% Joint 2: Shoulder elevation (rotation about y-axis in shoulder frame)
    % y_axis = [0; 1; 0];
    % J2 = cross(y_axis, v_total);
    % 
    % %% Joint 3: Shoulder rotation (rotation about upper arm axis)
    % J3 = cross(v_upper_norm, v_forearm);
    % 
    %% Joint 4: Elbow flexion (rotation about elbow axis)
    % elbow_axis = cross(v_upper_norm, v_forearm_norm);
    % 
    % if norm(elbow_axis) < 0.01  % Nearly aligned segments
    %     % Create artificial elbow axis
    %     elbow_axis = cross(v_upper_norm, [0; 0; 1]);
    %     if norm(elbow_axis) < 0.01
    %         elbow_axis = cross(v_upper_norm, [0; 1; 0]);
    %     end
    % end
    % elbow_axis = elbow_axis / norm(elbow_axis);
    % 
    % J4 = cross(elbow_axis, v_forearm);
    
    %% Assemble Jacobian matrix (3x4 for position only)
    % J = [J1, J2, J3, J4];
    % 
    % % Handle remaining singular cases
    % if any(isnan(J(:))) || any(isinf(J(:)))
    %     J = eye(3,4) * 0.01; % Small identity-like matrix
    % end
    % 
    % % Ensure J has reasonable conditioning
    % if cond(J) > 1e8  % More restrictive condition number
    %     J = J + 0.001 * eye(3,4); % Add small regularization
    % end

end