function A = zero_el(number)
%Function, excluding approximation and putting angles in area [-pi,pi]
            if abs(number)<1e-4
                number = 0;
            else 
                number = number;
            end
            
            
            if number>0
                while number>pi
                    number = number-pi;
                end
            end
            if number<0
                while number<-pi
                    number = number+pi;
                end
            end
            A = number;   
end

