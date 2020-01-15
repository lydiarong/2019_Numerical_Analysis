function [l,d]=cholesky(A) 
    dim=size(A); 
    n=dim(1); 
    for i=1:n 
        for j=1:n            
            if A(i,j)~=A(j,i) 
                error('The input matrix should be a Symteric Matrix!,see IDIT.m') 
            end 
        end 
    end 

    l=eye(n); 
    d=zeros(n); 
    g=zeros(n); 
    d(1,1)=A(1,1); 
    for i=2:n 
        for j=1:i-1 
            %calculate g(i,j) 
            temp=0; 
            for k=1:j-1 
                temp=temp+g(i,k)*l(j,k); 
            end 
             g(i,j)=A(i,j)-temp; 
            %calculate l(i,j) 
            l(i,j)=g(i,j)/d(j,j); 
        end %loop of j 
        %calculate d(i,i) 
        temp=0; 
        for k=1:i-1 
            temp=temp+g(i,k)*l(i,k); 
        end 
        d(i,i)=A(i,i)-temp; 
    end %loop of i 
end