function P=dummy_transition()

    P=zeros(6,6,2);
    P(1,2,2)=1;
    P(1,3,1)=1;
    P(2,4,1)=1;
    P(2,5,2)=1;
    P(3,5,2)=1;
    P(3,6,1)=1;
    P(4,4,1)=1;
    P(4,4,2)=1;
    P(5,5,1)=1;
    P(5,5,2)=1;
    P(6,6,1)=1;
    P(6,6,2)=1;
   