function plotEvolution(q, qmin, qmax)
    for i=1:6
        % Plot évolution
        subplot(3,2,i)
        plot(q(i,:), 'blue')
        hold on

        % Plot les bornes
        plot(ones(1,size(q,2)).*qmax(i), '--', 'color', 'red')
        plot(ones(1,size(q,2)).*qmin(i), 'red')
        
        max_int = max(qmax(i), max(q(i,:)));
        min_int = min(qmin(i), min(q(i,:)));
        ylim([min_int-0.5 max_int+0.5])

        legend(sprintf('q_{%d}',(i)),'q_{max}', 'q_{min}');
        grid on
        xlim([0 size(q,2)]);
        title(sprintf('q_{%d}',(i)));
        xlabel('temps (ms)') 
        ylabel('angle (rad)')
    end
end