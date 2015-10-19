%return 2x1 error vec, given u,v vec, R matrix, p_world and origin of world wrt cam
function e_vec = err_vec(f,u_vec,R_w_wrt_c,p_w_wrt_c,origin_w_wrt_c)
p_wrt_cam = R_w_wrt_c*p_w_wrt_c+origin_w_wrt_c;
pz = p_wrt_cam(3)
e_vec = (pz/f)*u_vec;
%compare this to p_wrt_cam
e_vec = e_vec -[p_wrt_cam(1);p_wrt_cam(2)]-[origin_w_wrt_c(1);origin_w_wrt_c(2)];
return
endfunction
