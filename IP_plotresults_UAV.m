%IP trajectory tracking for a UAV using differential flatness and feedback linearization
close all; clear all; clc

% load the reference state (position, velocity and heading) 
% these are arbirarly given, you can give your own references (which you designed in Lab Session 1)
load Positionref
load Veloref
load Psiref

% plot the reference position
figure; hold on; grid on
plot(positionref(2,:),positionref(3,:))
step=5;
%plots the direction of the UAV motion
quiver(positionref(2,1:step:end),positionref(3,1:step:end),dpositionref(1,1:step:end),dpositionref(2,1:step:end))
legend('reference trajectory', 'heading')

%%

%load the UAV simulink model
load_system('UAVFeedbackLinearize') 
set_param('UAVFeedbackLinearize', 'StopTime', num2str(positionref(1,end)))
sim('UAVFeedbackLinearize')
% Ajoutons les noms des axes
xlabel('Position en X (m)')
ylabel('Position en Y (m)')
title('Trajectoire de référence et direction du mouvement')

%%

% plot the position, velocity and the heading angle \Psi
figure; hold on; grid on
plot(position.signals.values(:,1),position.signals.values(:,2),'b')
plot(position.signals.values(:,3),position.signals.values(:,4),'r--')

legend('Trajectoire de référence','Trajectoire simulée ')
xlabel('Temps (s)')
ylabel('Position (m)')
title('Position simulée du UAV, pôle de placement = -10')
xlabel('Temps (s)')
ylabel('Position (m)')


figure; hold on; grid on
plot(psi.time, psi.signals.values(:,1), 'b')
plot(psi.time, psi.signals.values(:,2), 'r--')
legend('Angle de cap de référence', 'Angle de cap simulé')
xlabel('Temps (s)')
ylabel('Angle de cap \Psi (rad)')
title('Evolution de langle de cap')



% Plot de dVa
figure;
surf(dx, dy, dVa);
title('Variation de dVa');
xlabel('dx');
ylabel('dy');
zlabel('dVa');
colorbar; % Ajoute une barre de couleurs pour l'échelle

% Plot de tanPhi
figure;
surf(dx, dy, tanPhi);
title('Variation de tan(\phi)');
xlabel('dx');
ylabel('dy');
zlabel('tan(\phi)');
colorbar; % Ajoute une barre de couleurs pour l'échelle