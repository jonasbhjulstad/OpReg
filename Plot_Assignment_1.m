%Plotting assingment 1

simout = load('assignment1_reg_test');

fig1 = figure;
subplot(2,1,1);
plot(simout.ans(1,:), simout.ans(2,:)*180/pi)
hold on
plot(simout.ans(1,:), simout.ans(4,:)*180/pi)
hold off
legend('p_c', 'p')
xlabel('Time [s]')
ylabel('Angle [degrees]')
grid

subplot(2,1,2)
plot(simout.ans(1,:), simout.ans(3,:)*180/pi)
hold on
plot(simout.ans(1,:), simout.ans(5,:)*180/pi)
hold off
legend('e_c', 'e')
xlabel('Time [s]')
ylabel('Angle [degrees]')
grid

saveas(fig1,'pitch_elevation_plot_assignment1','epsc')