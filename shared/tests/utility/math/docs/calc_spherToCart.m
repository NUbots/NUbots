function out = calc_spherToCart(spher_coords)
for i = 1 : length(spher_coords)
    disp(sphericalToCartesian(spher_coords(i,:)));
end