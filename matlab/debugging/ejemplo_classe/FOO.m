classdef FOO
    %FOO Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        Property1
    end
    
    methods
        function obj = FOO(inputArg1,inputArg2)
            %FOO Construct an instance of this class
            %   Detailed explanation goes here
            obj.Property1 = inputArg1 + inputArg2;
        end
        
        function outputArg = sum(obj,inputArg)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            obj.Property1 = obj.Property1 + inputArg;
            outputArg = obj.Property1 + inputArg;
        end
        function outputArg = sub(obj,inputArg)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            obj.Property1 = obj.Property1 - inputArg;
            outputArg = obj.Property1 - inputArg;
        end
    end
end

