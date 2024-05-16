function crazyflie_show_sensor_data(t,a,g,h,p,v,lbd,printFigs,filename,ifig)

    % test input variables
    if ~exist('printFigs','var') | isempty(printFigs), printFigs = 0; end
    if ~exist('filename','var') | isempty(filename), filename = 'test'; end
    if ~exist('ifig','var') | isempty(ifig), ifig = 0; end

    is_ned = 0;

    % prepare data for plots
    NSim = size(t,2);
    
    % 3D plot
    ifig = ifig+1;
    figure(ifig);
    leg_str = '';
    hplt = plot3(p(1,:),p(2,:),p(3,:));
    hold on
    hini = plot3(p(1,1),p(2,1),p(3,1),'og','MarkerSize',2);
    hend = plot3(p(1,end),p(2,end),p(3,end),'xr','MarkerSize',2);
    hold off;
    grid on;
    axis equal;
    xlabel('$$p_1$$ [m]');
    ylabel('$$p_2$$ [m]');
    zlabel('$$p_3$$ [m]');
    if is_ned
        set(gca,'Zdir','reverse');
        set(gca,'Ydir','reverse');
    end
    title('Vehicle trajectory');
    legend([hplt,hini,hend],'traj','ini','end');
    print2pdf([filename '_traj'],printFigs);
    
    ifig = ifig+1;
    figure(ifig);
    for ivar = 1:3
        subplot(3,1,ivar);
        plot(t,p(ivar,:));
        hold off;
        grid on;
        ylabel(['$$p_' num2str(ivar) '(t) [m]$$']);
    end
    xlabel('$$t$$ [s]');
    title('Position');
    print2pdf([filename '_pos'],printFigs);

    ifig = ifig+1;
    figure(ifig);
    subplot(311);
    for ivar = 1:3
        subplot(3,1,ivar);
        plot(t,v(ivar,:));
        grid on;
        ylabel(['$$v_' num2str(ivar) '(t) [m/s]$$']);
    end
    xlabel('$$t$$ [s]');
    title('Linear velocity');
    print2pdf([filename '_vel'],printFigs);
    
    ifig = ifig+1;
    figure(ifig);
    subplot(311);
    for ivar = 1:3
        subplot(3,1,ivar);
        plot(t,lbd(ivar,:));
        grid on;
        ylabel(['$$\lambda_' num2str(ivar) '(t) [deg]$$']);
    end
    xlabel('$$t$$ [s]');
    title('Attitude (euler angles)');
    print2pdf([filename '_eul'],printFigs);
    
    ifig = ifig+1;
    figure(ifig);
    for ivar = 1:3
        subplot(3,1,ivar);
        plot(t,a(ivar,:));
        grid on;
        ylabel(['$$a_' num2str(ivar) '(t) [Gs]$$']);
    end
    xlabel('$$t$$ [s]');
    title('Acceleration');
    print2pdf([filename '_acc'],printFigs);
    
    ifig = ifig+1;
    figure(ifig);
    for ivar = 1:3
        subplot(3,1,ivar);
        plot(t,g(ivar,:));
        grid on;
        ylabel(['$$\omega_' num2str(ivar) '(t) [deg/s]$$']);
    end
    xlabel('$$t$$ [s]');
    title('Rate Gyros');
    print2pdf([filename '_gyro'],printFigs);


    ifig = ifig+1;
    figure(ifig);
    plot(t,h(:));
    grid on;
    ylabel(['$$h(t) [m]$$']);
    xlabel('$$t$$ [s]');
    title('ASL (barometer)');
    print2pdf([filename '_asl-baro'],printFigs);

end