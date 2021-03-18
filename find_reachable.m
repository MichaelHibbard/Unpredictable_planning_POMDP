function [reachable,G]=find_reachable(TF,init)
    nodes_1=zeros(size(TF,1)^2,1);
    nodes_2=zeros(size(TF,1)^2,1);

    P_dummy=sum(TF,3);
    counter=1;
    for s=1:size(TF,1)
        succ=find(P_dummy(s,:));
        nodes_1(counter:counter+length(succ)-1)=s*ones(length(succ),1);
        nodes_2(counter:counter+length(succ)-1)=succ;
        counter=counter+length(succ);
    end
    nodes_1(nodes_1==0) = [];
    nodes_2(nodes_2==0) = [];
    G=digraph(nodes_1,nodes_2);
    %plot(G)
    a=bfsearch(G,init);
    all_states=1:size(TF,1);
    reachable=intersect(a,all_states);
end