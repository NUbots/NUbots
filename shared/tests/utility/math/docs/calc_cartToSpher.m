function out = calc_cartToSpher(cart_coords)
for i = 1 : length(cart_coords)
disp(cartesianToSpherical(cart_coords(i,:)));
end
