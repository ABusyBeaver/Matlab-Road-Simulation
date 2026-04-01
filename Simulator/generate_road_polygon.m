function road_polygon = generate_road_polygon(left_border, right_border)
    % 生成车道多边形的 [x, y] 点
    road_polygon_x = [left_border(:,1); flipud(right_border(:,1))];
    road_polygon_y = [left_border(:,2); flipud(right_border(:,2))];
    road_polygon = [road_polygon_x, road_polygon_y];
end
