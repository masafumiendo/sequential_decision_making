function path_length = get_path_length(path)
    path_length = 0;
    for i=1:length(path)-1
        xs = path(i, 1); xe = path(i+1, 1);
        ys = path(i, 2); ye = path(i+1, 2);
        l = sqrt((xe - xs)^2 + (ye - ys)^2);
        path_length = path_length + l;
    end
end