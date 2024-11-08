function TransformMesh(mesh_h,tr)
    % Since no local tr is stored in mesh_h it is assuming that tr is
    % translating to be centered at tr's point in the global coordinate frame
    % and rotating wrt the global coordinate frame 
    vertices = get(mesh_h,'Vertices');
    verticeCount = size(vertices,1);
    
    % Translate first
    vertices = vertices + repmat(tr(1:3,4)',verticeCount,1);
    
    % Calculated center of the point cloud 
    midPoint = sum(vertices)/verticeCount;
    centeredVertices = vertices - repmat(midPoint,verticeCount,1);
    
    centeredRotatedVertices = centeredVertices * tr(1:3,1:3);%[vertices,ones(vertexCount,1)]';
    
    updatedVertices = centeredRotatedVertices + repmat(tr(1:3,4)',verticeCount,1);
    
    set(mesh_h,'Vertices',updatedVertices(:,1:3));
end