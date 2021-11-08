
%DIBUJAMOS LA IMAGEN ORIGINAL .STL
figure

path = 'pokemon3.stl';
[stlcoords] = READ_stl(path);
xco = squeeze( stlcoords(:,1,:) )';
yco = squeeze( stlcoords(:,2,:) )';
zco = squeeze( stlcoords(:,3,:) )';
[hpat] = patch(xco,yco,zco,'g');
axis equal

%CREAMOS EL VOXELIZADO DE LA MALLA DE TRIANGULOS 
[OUTPUTgrid,gridX, gridY, gridZ] = VOXELISE(120,120,120,path,'xyz');


%%VOXELS TO IMAGE

[faces,vertices] = CONVERT_voxels_to_stl('voxelOut.stl',OUTPUTgrid,gridX,gridY,gridZ,'ascii');
disp(faces);
%[cood] = [gridX,gridY,gridZ];
%disp(cood);
%[cood] = [0,0,0];
%disp(cood);

%%DRAW OUTPUT VOXELISE PROCESS!

figure;
subplot(1,3,1);
imagesc(squeeze(sum(OUTPUTgrid,1)));
colormap(gray(256));
xlabel('Z-direction');
ylabel('Y-direction');
axis equal tight

subplot(1,3,2);
imagesc(squeeze(sum(OUTPUTgrid,2)));
colormap(gray(256));
xlabel('Z-direction');
ylabel('X-direction');
axis equal tight

subplot(1,3,3);
imagesc(squeeze(sum(OUTPUTgrid,3)));
colormap(gray(256));
xlabel('Y-direction');
ylabel('X-direction');
axis equal tight

%%PART II: THINNING
skel = Skeleton3D((OUTPUTgrid));

figure();
%  col=[.7 .7 .8];
%  hiso = patch(isosurface([OUTPUTgrid],0),'FaceColor',col,'EdgeColor','none');
%  hiso2 = patch(isocaps([OUTPUTgrid],0),'FaceColor',col,'EdgeColor','none');
% %  disp(isosurface([OUTPUTgrid],0));
%  axis equal;axis off;
%  lighting phong;
%  isonormals(OUTPUTgrid,hiso);
%  alpha(0.5);
%  set(gca,'DataAspectRatio',[1 1 1])

camlight;
hold on;

%SKELETON DRAW THINNING

%TEST1
% figure
% [stlcoords] = READ_stl('voxelOut.stl');
% %disp(stlcoords);
% xco = squeeze( stlcoords(:,1,:) )';
% yco = squeeze( stlcoords(:,2,:) )';
% zco = squeeze( stlcoords(:,3,:) )';
% [COunt] = [0,0,0];

% 
% for i=1:12600
%     [COunt] = [COunt;xco(i),yco(i),zco(i)];
% end
% disp(length(COunt(:,1)));
% % [hpat] = patch(xco,yco,zco,'b');
% axis equal

w=size(skel,1);
l=size(skel,2);
h=size(skel,3);
[x,y,z]=ind2sub([w,l,h],find(faces(:)));
plot3(y,x,z,'square','Markersize',4,'MarkerFaceColor','b','Color','b');

set(gcf,'Color','white');
view(140,80)


%disp([bX,bY,bZ]);
% 
[bX,bY,bZ] = ind2sub([w,l,h],find(OUTPUTgrid(:)));
disp('INITIAL POINTS ');


%PART III: VOLUME ESTIMATE
%EN ESTE CASO RESULTA SENCILLO PODER CALCULAR LA DISTANCIA ENTRE DOS PUNTOS
%PUNTOS DE 3D 

backGROUND = [bX,bY,bZ];
finalGROUND = [x,y,z];
%SORTED
% backGS = sortrows(backGROUND,2);
% finalGS = sortrows(finalGROUND,2);

[result] = Distance(finalGROUND,backGROUND);
n = length(result(:,1));
%figure();
for i=1:n
    
    aux = result(i,:); 
    %disp(aux(4));
    center = aux([2,1,3]);
    radius = aux(4);
    theta=0:0.01:2*pi;
    v=null([aux(5),aux(6),aux(7)]);
    points=repmat(center',1,size(theta,2))+radius*(v(:,1)*cos(theta)+v(:,2)*sin(theta));
    plot3(points(1,:),points(2,:),points(3,:),'r-');
    %plot3(aux(2),aux(1),aux(3),'square','Markersize',4,'MarkerFaceColor','g','Color','g');
end 



%  for i=1:5
%      aux = finalGROUND(i,:); 
%      disp(aux);
%      aux = backGROUND(i,:);
%      disp(aux);
%  end

% for i=1:n
%     aux = result(i,:); 
%     center = aux([1,2,3]);
%     radius = aux(4);
%     theta=0:0.01:2*pi;
%     v=null([0,0,1]);
%     points=repmat(center',1,size(theta,2))+radius*(v(:,1)*cos(theta)+v(:,2)*sin(theta));
%     plot3(points(1,:),points(2,:),points(3,:),'r-');
% end 


%PART IV: GRAPH REPRESENTATION 

w = size(skel,1);
l = size(skel,2);
h = size(skel,3);

% % Detectemos el inicio
[~,node,link] = Skel2Graph3D(skel,0);

% % Tamaño del grafo total
wl = sum(cellfun('length',{node.links}));

skel2 = Graph2Skel3D(node,link,w,l,h);
[~,node2,link2] = Skel2Graph3D(skel2,0);

% %  Tamaño del grafo total generado
wl_new = sum(cellfun('length',{node2.links}));

% % iterate the same steps until network length changed 
while(wl_new~=wl)

    wl = wl_new;   
    
     skel2 = Graph2Skel3D(node2,link2,w,l,h);
     [A2,node2,link2] = Skel2Graph3D(skel2,0);

     wl_new = sum(cellfun('length',{node2.links}));

end;

% %%SHOW GRAPH REPRESENTATION
figure();
hold on;
for i=1:length(node2)
    x1 = node2(i).comx;
    y1 = node2(i).comy;
    z1 = node2(i).comz;
    
    if(node2(i).ep==1)
        ncol = 'c';
    else
        ncol = 'y';
    end;
    
    for j=1:length(node2(i).links)    % draw all connections of each node
        if(node2(node2(i).conn(j)).ep==1)
            col='k'; % branches are black
        else
            col='r'; % links are red
        end;
        if(node2(i).ep==1)
            col='k';
        end;

        
%         % DIBUHAMOS LOS VERTICES DEL GRAFO GENERADO
        for k=1:length(link2(node2(i).links(j)).point)-1            
            [x3,y3,z3]=ind2sub([w,l,h],link2(node2(i).links(j)).point(k));
            [x2,y2,z2]=ind2sub([w,l,h],link2(node2(i).links(j)).point(k+1));
            line([y3 y2],[x3 x2],[z3 z2],'Color',col,'LineWidth',2);
        end;
    end;
    
%     % TODO LOS NODOS O VERTICIES SERAN DE COLRO AMARILLO
    plot3(y1,x1,z1,'o','Markersize',8,...
        'MarkerFaceColor',ncol,...
        'Color','k');
end;
axis image;axis off;
set(gcf,'Color','white');
drawnow;
view(-17,46);

% %PRUNING
% 
% %DADO EL GRAFO APLICAMOS UN ALMORITMO EN LOS NODOS PARA ENCONTRAR EL CAMINO
% %MAS CORTO POSIBLE
% 
% 
% 
% %PART V: SEGEMTANCION ESPECTRAL
% 
% 
% %PART VI: HIGH FREQUENCIES REMOVAL
% 
% 
% %PART VII:  RETRIEVAL ORIGINAL REGION 









