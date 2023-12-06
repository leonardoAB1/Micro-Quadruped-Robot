function plotRoboticLeg(points)
    % Extract coordinates from the input vector
    x00 = points(1);
    y00 = points(2);
    z00 = points(3);
    x0 = points(4);
    y0 = points(5);
    z0 = points(6);
    x1 = points(7);
    y1 = points(8);
    z1 = points(9);
    x2 = points(10);
    y2 = points(11);
    z2 = points(12);
    x_ab = points(13);
    y_ab = points(14);
    z_ab = points(15);
    x_bc = points(16);
    y_bc = points(17);
    z_bc = points(18);

    % Plot the robotic leg

    plot3(x00, y00, z00, 'k.');

    plot3(x0, y00, z00, 'k.');
    line([x00 x0], [y00 y00], [z00 z00],'Color', 'k');
    line([x0 x0], [y00 y0], [z00 z0],'Color', 'k');

    plot3(x0, y0, z0, 'k.');
    plot3(x1, y1, z1, 'r.');
    plot3(x2, y2, z2, 'b.');
    line([x0 x1], [y0 y1], [z0 z1], 'Color', 'r');
    line([x1 x2], [y1 y2], [z1 z2], 'Color', 'b');

    plot3(x_ab, y_ab, z_ab, 'm.');
    plot3(x_bc, y_bc, z_bc, 'm.');
    line([x1 x_bc], [y1 y_bc], [z1 z_bc], 'Color', 'm');
    line([x0 x_ab], [y0 y_ab], [z0 z_ab], 'Color', 'm');
    line([x_ab x_bc], [y_ab y_bc], [z_ab z_bc], 'Color', 'm');
    
end
