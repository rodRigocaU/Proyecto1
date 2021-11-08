function [result] = Distance(mat1,mat2)
persistent temp;

n = length(mat1(:,1));
m = length(mat2(:,1));

for i=1:10
    aux = mat1(i,:); 
    tempXF = aux(1);
    tempYF = aux(2);
    tempZF = aux(3);
    disM = 1000;
    %disp(disM);
    
    
    for j=1:m
        aux = mat2(j,:);
        tempXB = aux(1);
        tempYB = aux(2);
        tempZB = aux(3);

        auxDis = sqrt((tempZF - tempZB)^(2) + (tempXF - tempXB)^(2) + (tempYF - tempYB)^(2));
        %disp(auxDis);
        if auxDis < disM && auxDis > 7
            disM = auxDis;
            xX = (tempXB - tempXF);
            yY = (tempYB - tempYF);
            zZ = (tempZB - tempZF);
            
            temp = [disM,xX,yY,zZ];
            
            %[temPP2] = [disM,xX,yY,zZ];
            %disp(temPP2);
            %disp(temPP2([1,2,3,4]));  
        end
        %disp(temp);
    end
%     disp(temp);
%     disp('DEAD VARIABLE');
%     disp(temp);
    temPP = [tempXF,tempYF,tempZF,temp(1),temp(2),temp(3),temp(4)];
    %disp(temPP);
    if i == 1 
        [result] = [temPP];
    else
        [result] = [result;temPP];
    end
    %disp([temPP]);
%     result = [result;temPP,temPP2];
  
end

% for i=1:n
%     aux = result(i,:); 
%     disp(aux(4));
% end 

end

