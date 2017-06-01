classdef linkpart < dynamicprops
    %LINKPART Class for linkage part in mechanism
    %   Property names are self-explanatory, see OneNote for more details.
    %   LINKPART has dynamic propeties in order to accomodate part-specific
    %   demands. May represent either a part or a contact point between
    %   parts.
    %   
    %   Matthew R. Bonanni
    %   04-2017
    
    properties
        x
        y
        x_i
        x_f
        y_i
        y_f
        theta
        end_rad
        end_dist
        sweep
    end
    
    methods
    end
    
end