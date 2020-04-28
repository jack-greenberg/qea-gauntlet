function gauntlet()
    neato = NEATO;
    
    disp(neato);
    
    [r, theta] = neato.LIDAR_scan(neato);
    disp(r);
    disp(theta);
    
    neato.destroy();
end