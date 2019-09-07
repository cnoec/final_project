%       analisi triettoria

%       loading workspace

% divisione traiettoria

% global Par_1 Par_2 Par_3 Par_4 Par_5 Par_6 Par_7 Par_8

path                    =   pwd;
addpath('Functions');
addpath('Model');
addpath('Mat_Data\');
addpath('Functions\unc_optimization');
addpath('Functions\con_optimization');
addpath('Functions\track');
addpath('Functions\cost_function');

n_states = T_end/Ts_sim;

Par_3               =   CircleFitByTaubin(outerBoundary(30:98,:));
Par_4               =   CircleFitByTaubin(innerBoundary(30:98,:));

Par_7               =   CircleFitByTaubin(outerBoundary(158:228,:));
Par_8               =   CircleFitByTaubin(innerBoundary(158:228,:));

Par_1               =   -Par_3(3)+Par_4(2);
Par_2               =   -Par_4(3)+Par_4(2);

Par_5               =   Par_3(3)+Par_4(2);
Par_6               =   Par_4(3)+Par_4(2);

h = zeros(n_states,1);

for ind = 1:n_states

    state = [xi_an(1,ind) xi_an(2,ind)];      %punto da verificare
    if state(1) > Par_4(1)

        centre_x = mean([Par_3(1) Par_4(1)]);
        centre_y = mean([Par_3(2) Par_4(2)]);
    
        if state(2) <= centre_y
            h(ind) = 1;
        else
            h(ind) = 2;
        end
         
    elseif state(1) > Par_8(1) && state(1) <= Par_4(1)
        if state(2) > Par_4(2)      % settore C
            h(ind) = 3;
        else
            h(ind) = 6;
%         disp('F');      % settore F
        end
      
    elseif state(1) < Par_8(1)
        centre_x = mean([Par_7(1) Par_8(1)]);
        centre_y = mean([Par_7(2) Par_8(2)]);
        if state(2) <= centre_y
            h(ind) = 5;
        else
            h(ind) = 4;
        end
    end
end


%%

[xi_an, ~, ~, ~] = trajectory_generation_cc(u_opt, xi0, T_end, 0.1,1e-2);


beta = xi_an(4,:)*180/pi;
figure('Name', 'side slip angle', 'NumberTitle', 'off')
plot(t_vec, beta, t_vec, -5*ones(n_states), t_vec, 5*ones(n_states)), grid on;

    
c1 = 0;
c2 = 0;
c3 = 0;
c4 = 0;
c5 = 0;
c6 = 0;

xi_1 = zeros(6,1);
xi_2 = zeros(6,1);
xi_3 = zeros(6,1);
xi_4 = zeros(6,1);
xi_5 = zeros(6,1);
xi_6 = zeros(6,1);
    
figure('Name', 'Trajectory', 'NumberTitle', 'off')
% subplot 211
plot(   innerBoundary(:,1),innerBoundary(:,2),'black',outerBoundary(:,1),...
        outerBoundary(:,2),'black'),grid on, axis equal, hold on
    
for ind = 1:n_states
    if h(ind) == 1
        c1 = c1 + 1;
%         plot(xi(1,ind),xi(2,ind),'.r')
        xi_1(:,c1) = xi_an(:,ind);
        
    elseif h(ind) == 2
        c2 = c2 + 1;
        if beta(ind) < -5 || beta(ind) > 5
            plot(xi_an(1, ind), xi_an(2,ind), '.r')
            hold on
        else
            plot(xi_an(1, ind), xi_an(2,ind), '.blue')
            hold on
        end
%         plot(xi(1,ind),xi(2,ind),'.g')
        xi_2(:,c2) = xi_an(:,ind);
        
    elseif h(ind) == 3
        c3 = c3 + 1;
%         plot(xi(1,ind),xi(2,ind),'.b')
        xi_3(:,c3) = xi_an(:,ind);
        
    elseif h(ind) == 4
        c4 = c4 + 1;
%         plot(xi(1,ind),xi(2,ind),'.k')
        xi_4(:,c4) = xi_an(:,ind);
        
    elseif h(ind) == 5
        c5 = c5 + 1;
        if beta(ind) < -5 || beta(ind) > 5
            plot(xi_an(1, ind), xi_an(2,ind), '.r')
            hold on
        else
            plot(xi_an(1, ind), xi_an(2,ind), '.blue')
            hold on
        end
%         plot(xi(1,ind),xi(2,ind),'.y')
        xi_5(:,c5) = xi_an(:,ind);
        
    elseif h(ind) == 6
        c6 = c6 + 1;
%         plot(xi(1,ind),xi(2,ind),'.m')
        xi_6(:,c6) = xi_an(:,ind);
    end
    hold on
end

title('Blue: small \beta, Red: high \beta');

%% settore 1

beta = xi_1(4, :)*180/pi;
r = xi_1(6,:);
r_expected = xi_1(3,:)/Par_4(3);

figure('Name', 'Settore 1', 'NumberTitle', 'off')
subplot 211
plot(1:length(xi_1), beta, 1:length(xi_1), 3*ones(length(xi_1),1), 1:length(xi_1), -3*ones(length(xi_1),1))
grid on;
subplot 212
plot(1:length(xi_1), r, 1:length(xi_1), r_expected), grid on;




%% settore 2

beta = xi_2(4, :)*180/pi;
r = xi_2(6,:);
r_expected = xi_2(3,:)/Par_4(3);

figure('Name', 'Settore 2', 'NumberTitle', 'off')
subplot 211
plot(1:length(xi_2), beta, 1:length(xi_2), 3*ones(length(xi_2),1), 1:length(xi_2), -3*ones(length(xi_2),1))
grid on;
subplot 212
plot(1:length(xi_2), r, 1:length(xi_2), r_expected), grid on;


figure
plot(   innerBoundary(:,1),innerBoundary(:,2),'black',outerBoundary(:,1),...
        outerBoundary(:,2),'black'),grid on, axis equal, hold on

for ind = 1:length(xi_2)
    if beta(ind) < -5 || beta(ind) > 5
        plot(xi_2(1, ind), xi_2(2,ind), '.r')
        hold on
    else
        plot(xi_2(1, ind), xi_2(2,ind), '.blue')
        hold on
    end
end

%%
% %% settore 3
% 
% beta = xi_3(4, :)*180/pi;
% 
% figure('Name', 'Settore 3', 'NumberTitle', 'off')
% 
% plot(1:length(xi_3), beta, 1:length(xi_3), 3*ones(length(xi_3),1), 1:length(xi_3), -3*ones(length(xi_3),1))
% grid on;
% 
% figure
% plot(   innerBoundary(:,1),innerBoundary(:,2),'black',outerBoundary(:,1),...
%         outerBoundary(:,2),'black'),grid on, axis equal, hold on
% 
% for ind = 1:length(xi_3)
%     if beta(ind) < -5 || beta(ind) > 5
%         plot(xi_3(1, ind), xi_3(2,ind), '.r')
%         hold on
%     else
%         plot(xi_3(1, ind), xi_3(2,ind), '.blue')
%         hold on
%     end
% end

%% settore 4

beta = xi_4(4, :)*180/pi;
r = xi_4(6,:);
r_expected = xi_4(3,:)/Par_8(3);

figure('Name', 'Settore 4', 'NumberTitle', 'off')
subplot 211
plot(1:length(xi_4), beta, 1:length(xi_4), 5*ones(length(xi_4),1), 1:length(xi_4), -5*ones(length(xi_4),1))
grid on; title('\beta [rad]');
subplot 212
plot(1:length(xi_4), r, 1:length(xi_4), r_expected), grid on, title('r [rad/s]');

figure
plot(   innerBoundary(:,1),innerBoundary(:,2),'black',outerBoundary(:,1),...
        outerBoundary(:,2),'black'),grid on, axis equal, hold on

for ind = 1:length(xi_4)
    if beta(ind) < -5 || beta(ind) > 5
        plot(xi_4(1, ind), xi_4(2,ind), '.r')
        hold on
    else
        plot(xi_4(1, ind), xi_4(2,ind), '.blue')
        hold on
    end
end

%% settore 5

beta = xi_5(4, :)*180/pi;
r = xi_5(6,:);
r_expected = xi_5(3,:)/Par_8(3);

figure('Name', 'Settore 5', 'NumberTitle', 'off')
subplot 211
plot(1:length(xi_5), beta, 1:length(xi_5), 5*ones(length(xi_5),1), 1:length(xi_5), -5*ones(length(xi_5),1))
grid on; title('\beta [rad]');
subplot 212
plot(1:length(xi_5), r, 1:length(xi_5), r_expected), grid on, title('r [rad/s]');

figure
plot(   innerBoundary(:,1),innerBoundary(:,2),'black',outerBoundary(:,1),...
        outerBoundary(:,2),'black'),grid on, axis equal, hold on

for ind = 1:length(xi_5)
    if beta(ind) < -5 || beta(ind) > 5
        plot(xi_5(1, ind), xi_5(2,ind), '.r')
        hold on
    else
        plot(xi_5(1, ind), xi_5(2,ind), '.blue')
        hold on
    end
end

%%
