function [ ] = plot_estimates_vs_true( anchors,pos_nodes,est_nodes,dim )
%plot_estimates_vs_true plot true positions and computed estimates

figure()
hold on

if (dim == 2)
    sz = size(pos_nodes);
    for i=1:dim:(sz(1))
        plot_x = pos_nodes(i,:);
        plot_y = pos_nodes(i+1,:);
        if(i==1)
            p1= plot(plot_x, plot_y,'b');
        elseif (i==3)
            plot(plot_x, plot_y,'b');
        end
    end
    %ANchors
    sz = size(anchors);
    for i=1:dim:(sz(1))
        plot_x = anchors(i,:);
        plot_y = anchors(i+1,:);
        if(i==1)
            p2 = plot(plot_x, plot_y,'g--');
        elseif (i==3)
            p3 = plot(plot_x, plot_y,'k:');
        elseif(i==5)
             plot(plot_x, plot_y,'c-');
        end
        hold on
    end
    % Estimated nodes
    sz = size(est_nodes);
    for i=1:dim:(sz(1))
        plot_x = est_nodes(i,:);
        plot_y = est_nodes(i+1,:);
        if(i==1)
            p4 = plot(plot_x, plot_y,'r');
        elseif (i==3)
            plot(plot_x, plot_y,'r');
        end
    end
    xlabel('x[m]')
    ylabel('y[m]')
    v = [p1 p2 p3 p4 ]';
    legend(v,'Nodes true position', 'Anchor 1', 'Anchor 2', 'Nodes estimated position' );

end

if (dim == 3)
    sz = size(pos_nodes);
    for i=1:dim:(sz(1))
        plot_x = pos_nodes(i,:);
        plot_y = pos_nodes(i+1,:);
        plot_z = pos_nodes(i+2,:);
        p1 = plot3(plot_x, plot_y,plot_z,'b');
    end
    sz = size(anchors);
    for i=1:dim:(sz(1))
        plot_x = anchors(i,:);
        plot_y = anchors(i+1,:);
        plot_z = anchors(i+2,:);
        if(i==1)
            p2 = plot3(plot_x, plot_y,plot_z,'g');
        elseif (i==4)
            p3 = plot3(plot_x, plot_y,plot_z,'k');
        elseif(i==7)
            p4 = plot3(plot_x, plot_y,plot_z,'color',[0, 0.5, 0]);
        end
    end
    sz = size(est_nodes);
    for i=1:dim:(sz(1))
        plot_x = est_nodes(i,:);
        plot_y = est_nodes(i+1,:);
        plot_z = est_nodes(i+2,:);
        
        p5 = plot3(plot_x, plot_y,plot_z,'r');
    end
    xlabel('x[m]')
    ylabel('y[m]')
    zlabel('z[m]')
       v = [p1 p2 p3 p4 p5]';
    legend(v,'Nodes true position', 'Anchor 1', 'Anchor 2','Anchor 3', 'Nodes estimated position' );
end

title('Accurate vs Estimated positions')

end

