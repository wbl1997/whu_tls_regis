function check_o3d_downsample(lasFile, dsPCD)
%CHECKO3DDOWNSAMPLE Compare voxel-downsampled LAS vs. existing PCD
%   CHECKO3DDOWNSAMPLE(lasFile, dsPCD) reads the original las or pcd file,
%   if las, downsamples it using gridNearest with a 0.05 m voxel,
%   loads the provided downsampled PCD, and displays them together.
    close all
    [p, n, e] = fileparts(lasFile);
    ext = lower(e);
    
    if strcmp(ext, '.las')
        % Read and downsample the LAS
        lasReader = lasFileReader(lasFile);
        pcOrig    = readPointCloud(lasReader);
        gridStep  = 0.05;  % voxel size in meters
        pcOrigDS  = pcdownsample(pcOrig, "gridNearest", gridStep);
    elseif strcmp(ext, '.pcd')
        % Read the pre‐downsampled PCD
        pcOrigDS  = pcread(lasFile);
    else
        error("Unsupported file extension: %s", ext);
    end

    % 2) Load downsampled PCD
    pcDown    = pcread(dsPCD);

    figure('Color','w');
    pcshowpair(pcOrigDS, pcDown);
    title("Original LAS (blue) & Downsampled PCD (red)");
    axis equal;
    xlabel("X (m)");  ylabel("Y (m)");  zlabel("Z (m)");
    grid on;
end