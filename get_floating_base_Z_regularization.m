function header = get_floating_base_Z_regularization(varargin)
            Parser = inputParser;
            Parser.FunctionName = 'get_Z_reguliration';
            Parser.addOptional('indices', []);
            Parser.addOptional('nodes_position', []);
            Parser.parse(varargin{:});

            indices = Parser.Results.indices;
            nodes_position = Parser.Results.nodes_position;

    function c = cost(x)
        nodes = reshape(x, 3, []);
        difference = abs(nodes(3, indices) - nodes_position(3, indices));
        c = std(difference);
    end
    
    header = @cost;
end