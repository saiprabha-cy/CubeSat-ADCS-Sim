classdef plotting_helpers
    % PLOTTING_HELPERS  Shared plotting utilities for the ADCS sim scripts.
    %
    % Every main_*.m script currently repeats near-identical subplot/
    % legend/grid boilerplate five times over. This file exists purely to
    % remove that duplication -- it has no effect on simulation results,
    % it only cleans up how they're plotted and saved.
    %
    % Usage from any sim script:
    %   plotting_helpers.plot_series(t, omega_hist, {'\omega_x','\omega_y','\omega_z'}, ...
    %       'Time (s)', '\omega (rad/s)', 'Angular velocity');
    %   plotting_helpers.add_limit_line(h_max, '-h_{max}');
    %   plotting_helpers.save_and_report(gcf, '../results/figures/my_result.png');

    methods (Static)

        function plot_series(t, data, series_labels, xlabel_str, ylabel_str, title_str)
            % Plot one or more time-series columns of `data` against `t`
            % in the CURRENT axes (call subplot(...) before this if needed).
            %
            %   t             : Nx1 time vector
            %   data          : NxM matrix, one column per series
            %   series_labels : cell array of M strings for the legend,
            %                   or [] / {} to skip the legend entirely
            %   xlabel_str, ylabel_str, title_str : plot labels

            plot(t, data, 'LineWidth', 1.3);
            xlabel(xlabel_str);
            ylabel(ylabel_str);
            title(title_str);
            grid on;
            if ~isempty(series_labels)
                legend(series_labels, 'Location', 'best');
            end
        end

        function plot_scalar(t, data, xlabel_str, ylabel_str, title_str, line_color)
            % Plot a single scalar time series (e.g. |omega|, attitude
            % error angle) with a distinct default color so magnitude/
            % error plots are visually distinguishable from the 3-axis
            % vector plots at a glance across figures.
            if nargin < 6 || isempty(line_color)
                line_color = [0.85 0.1 0.1];   % red, matches the convention
                                                 % already used for |omega|
                                                 % and error-angle plots
            end
            plot(t, data, 'LineWidth', 1.5, 'Color', line_color);
            xlabel(xlabel_str);
            ylabel(ylabel_str);
            title(title_str);
            grid on;
        end

        function add_limit_line(limit_value, label_str)
            % Add a dashed horizontal reference line to the CURRENT axes
            % for a saturation/threshold limit (h_max, tau_max, a settling
            % threshold, etc.) -- wraps yline with the styling already
            % used consistently across main_detumble.m / main_pointing.m /
            % main_disturbance_rejection*.m, so future plots match them
            % without re-typing the style options each time.
            hold on;
            yline(limit_value, '--k', label_str);
        end

        function save_and_report(fig_handle, filepath)
            % Save `fig_handle` to `filepath` and print a confirmation
            % line matching the "Figure saved to ..." message already
            % used at the end of every main_*.m script.
            saveas(fig_handle, filepath);
            fprintf('\nFigure saved to %s\n', filepath);
        end

    end
end