%% Gavin: This file is not read for release

% First load the mat file into matlab, which should be called shapeModel
data.vertex.x = shapeModel(7).vertex(:,1);
data.vertex.y = shapeModel(7).vertex(:,2);
data.vertex.z = shapeModel(7).vertex(:,3);
faceZeroIndex = shapeModel(7).face-1;
data.face.vertex_indices = num2cell(faceZeroIndex',[1,3]);
plywrite(data,'testLink7.ply','ascii')

%Can ONLY open in Blender (since the plywrite says the faces are char
%rather than int or unit

% Save from blender so it can be opened in Meshlab, or work on the file in
% Blender

% export

%Then open in Matlab
data = plyread('testLink7BlenderStretched.ply');

% To update the vertice data you could do this
shapeModel(7).vertex = [data.vertex.x(:),data.vertex.y(:),data.vertex.z(:)]

%AND/OR to change the face data
shapeModel(7).face = cell2mat(f.face.vertex_indices)+1

%Then in matlab right click and save the shapeModel as the updated .mat
%file