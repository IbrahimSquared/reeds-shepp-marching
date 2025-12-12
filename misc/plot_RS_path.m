function full_black_path = plot_RS_path(pathSegObj,t,x,y,plot_black,plot_red,plot_blue,z,step)
    full_black_path = [];
    r = pathSegObj.MinTurningRadius;
    for k = 1:length(pathSegObj.MotionTypes)
        motionType = pathSegObj.MotionTypes{k};
        motionLength = pathSegObj.MotionLengths(k);
        motionDirection = pathSegObj.MotionDirections(k);
        if motionType == 'N'
            break
        elseif motionType == 'L'
            cxl = x - r * cos(t-pi/2);
            cyl = y - r * sin(t-pi/2);

            tf = t + motionDirection * motionLength/r;
            if abs(motionLength) < 1e-6
                continue;
            end
            c = 1;
            blue_pts = [];
            black_pts = [];
            for t = t:motionDirection*step:tf
                if plot_blue
                    xx = cxl + 2 * r * cos(t-pi/2);
                    yy = cyl + 2 * r * sin(t-pi/2);
                    blue_pts(c,:) = [xx yy z];
                end
                if plot_black
                    xx = cxl + 1 * r * cos(t-pi/2);
                    yy = cyl + 1 * r * sin(t-pi/2);
                    black_pts(c,:) = [xx yy t];
                end
                c = c + 1;
            end
            if plot_blue
                plot3(blue_pts(:,1), blue_pts(:,2), blue_pts(:,3), 'color', 'blue', 'LineWidth', 2);
            end
            if plot_black
                % plot3(black_pts(:,1), black_pts(:,2), black_pts(:,3), 'color', 'k', 'LineWidth', 2);
                % append to full_black_path
                full_black_path = [full_black_path; black_pts];
            end
            x = cxl + r * cos(t-pi/2);
            y = cyl + r * sin(t-pi/2);
            
        elseif motionType == 'R'
            cxr = x + r * cos(t-pi/2);
            cyr = y + r * sin(t-pi/2);

            tf = t - motionDirection * motionLength/r;
            if abs(motionLength) < 1e-6
                continue;
            end
            c = 1;
            red_pts = [];
            black_pts = [];
            for t = t:-motionDirection*step:tf
                if plot_red
                    xx = cxr - 2 * r * cos(t-pi/2);
                    yy = cyr - 2 * r * sin(t-pi/2);
                    red_pts(c,:) = [xx yy z];
                end
                if plot_black
                    xx = cxr - 1 * r * cos(t-pi/2);
                    yy = cyr - 1 * r * sin(t-pi/2);
                    black_pts(c,:) = [xx yy t];
                end
                c = c + 1;
            end
            if plot_red
                plot3(red_pts(:,1), red_pts(:,2), red_pts(:,3), 'color', 'red', 'LineWidth', 2);
            end
            if plot_black
                % plot3(black_pts(:,1), black_pts(:,2), black_pts(:,3), 'color', 'k', 'LineWidth', 2);
                % append to full_black_path
                full_black_path = [full_black_path; black_pts];
            end
            x = cxr - r * cos(t-pi/2);
            y = cyr - r * sin(t-pi/2);
        elseif motionType == 'S'
            x_start = x;
            y_start = y;

            c = 1;
            red_pts = [];
            black_pts = [];
            blue_pts = [];
            % nb_of_steps = 1/step;
            nb_of_steps = 1/step;
            for itr = 0:nb_of_steps
                if plot_black
                    d_ = itr/nb_of_steps * motionLength * motionDirection;
                    x = x_start + d_ * cos(t);
                    y = y_start + d_ * sin(t);
                    black_pts(c,:) = [x y t];
                end
                if plot_red
                    cxr = x + r * cos(t-pi/2);
                    cyr = y + r * sin(t-pi/2);
                    xx = cxr - 2 * r * cos(t-pi/2);
                    yy = cyr - 2 * r * sin(t-pi/2);
                    red_pts(c,:) = [xx yy z];
                end
                if plot_blue
                    cxl = x - r * cos(t-pi/2);
                    cyl = y - r * sin(t-pi/2);
                    xx = cxl + 2 * r * cos(t-pi/2);
                    yy = cyl + 2 * r * sin(t-pi/2);
                    blue_pts(c,:) = [xx yy z];
                end
                c = c + 1;
            end
            if plot_red
                plot3(red_pts(:,1), red_pts(:,2), red_pts(:,3), 'color', 'red', 'LineWidth', 2);
            end
            if plot_black
                % plot3(black_pts(:,1), black_pts(:,2),  black_pts(:,3), 'color', 'k', 'LineWidth', 2);
                % append to full_black_path
                full_black_path = [full_black_path; black_pts];
            end
            if plot_blue
                plot3(blue_pts(:,1), blue_pts(:,2), blue_pts(:,3), 'color', 'blue', 'LineWidth', 2); 
            end
        end
    end
    full_black_path(:,3) = wrapTo2Pi(full_black_path(:,3));
end
