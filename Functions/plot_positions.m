function [ ] = plot_positions( anchors,pos_nodes,dim )
%plot_positions plots anchor and node trajectory in same plot

figure()
hold on

if (dim == 2)
    sz = size(pos_nodes);
    for i=1:dim:(sz(1))
        plot_x = pos_nodes(i,:);
        plot_y = pos_nodes(i+1,:);
        p1(i) = plot(plot_x, plot_y,'b-');
        plot(plot_x(1),plot_y(1),'bo')
    end
    sz = size(anchors);
    for i=1:dim:(sz(1))
        if(i==1)
            plot_x = anchors(i,:);
            plot_y = anchors(i+1,:);
            p3 = plot(plot_x, plot_y,'r--');
            plot(plot_x(1),plot_y(1),'ro')
        else
            plot_x = anchors(i,:);
            plot_y = anchors(i+1,:);
            p4 =plot(plot_x, plot_y,'k:');
            plot(plot_x(1),plot_y(1),'ko')
        end
    end
    xlabel('x[m]')
    ylabel('y[m]')
    v = [p1(1) p1(3) p3 p4 ]';
    legend(v,'Node 1', 'Node 2', 'Anchor 1', 'Anchor 2');
end

if (dim == 3)
    sz = size(pos_nodes);
    for i=1:dim:(sz(1))
        plot_x = pos_nodes(i,:);
        plot_y = pos_nodes(i+1,:);
        plot_z = pos_nodes(i+2,:);
        if(i==1)
            p1 = plot3(plot_x, plot_y,plot_z,'b');
        elseif(i==4)
            p2 = plot3(plot_x, plot_y,plot_z,'c');
        end
    end
    sz = size(anchors);
    for i=1:dim:(sz(1))
        plot_x = anchors(i,:);
        plot_y = anchors(i+1,:);
        plot_z = anchors(i+2,:);
        if(i==1)
            p3 = plot3(plot_x, plot_y,plot_z,'r');
        elseif(i==4)
            p4 = plot3(plot_x, plot_y,plot_z,'g');
        elseif(i==7)
            p5 = plot3(plot_x, plot_y,plot_z,'k');
        end
    end
    xlabel('x[m]')
    ylabel('y[m]')
    zlabel('z[m]')
     v = [p1 p2 p3 p4 p5]';
    legend(v,'Node 1', 'Node 2', 'Anchor 1', 'Anchor 2','Anchor 3');
end

title('Anchors and Nodes true positions')

end

