function header = get_floating_base_Z_reguliration(varargin)
            Parser = inputParser;
            Parser.FunctionName = 'get_Z_reguliration';
            Parser.addOptional('indices', []);
            Parser.addOptional('nodes_position', []);
            Parser.parse(varargin{:});

            indices = Parser.Results.indices;
            nodes_position = Parser.Results.nodes_position;

    function c = cost(x)
        nodes = reshape(x, 3, []);
        error = abs(nodes(3, indices) - nodes_position(3, indices));
        c = std(error);
    end
    
    header = @cost;
end