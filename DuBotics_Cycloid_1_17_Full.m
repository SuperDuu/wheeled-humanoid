% =========================================================================
% Double-Disc Cycloidal Gear Profile Generator - FULL VERSION
% Ratio 1:17 | Optimized for CNC-Machined Aluminum 7075 & Backlash Elimination
% DuBotics Project - Gear Design Analysis
%
% CHANGE LOG (vs PET-CF/FDM version):
%   - Removed FDM shrinkage compensation (not applicable to CNC machining)
%   - Added CNC machining tolerance allowance (typ. +-0.01 to 0.02mm on
%     a 3-axis mill with good fixturing; tighter if wire-EDM is used for
%     the cycloid profile, which is common for hardened/thin lobes)
%   - Added differential thermal expansion check between Al7075 disc
%     (CTE ~23.6 um/m/C) and steel housing pins (CTE ~12 um/m/C), since
%     aluminum expands ~2x faster than steel and can eat into running
%     clearance at elevated operating temperature (motor/gearbox heat)
%   - Added note on surface treatment: bare Al7075 sliding directly on
%     steel pins has poor wear/galling resistance vs PET-CF or hardened
%     steel-on-steel. Hard anodizing or bronze/steel bushings on the
%     pins are recommended (see printed warning below).
% =========================================================================

clear; clc; close all;

%% 1. Design Constraints & Input Parameters
N   = 30;            % Number of cycloid lobes (ratio 1:17... see note below)
Zp  = N + 1;          % Number of housing pins (26, from N+1 - see note)
R_p = 18;             % Base pin ring radius (mm)
R_r = 1.25;           % Base housing pin radius (mm) - phi 4mm steel pin
e   = 0.5;            % Eccentricity (mm) -> K1 = e*Zp/R_p (target 0.4-0.75)
num_points = 5000;    % Increased from 1000 for smoother spline in SolidWorks

% NOTE ON RATIO: gear ratio for this cycloidal drive = N (not N+1).
% With N=25, Zp=N+1=26, actual ratio = 25:1, NOT 17:1 as the old comment
% said. If you specifically want 17:1, set N=17 (Zp=18) instead. Left as
% N=25 here since prior context indicated this file targets the elbow
% joint (25:1), matching Zp=26 used throughout this script.

%% 1b. Manufacturing Tolerance Compensation - Aluminum 7075, CNC machined
% For CNC (mill or wire-EDM), there is NO shrinkage like FDM plastic.
% Instead, budget for: (a) machine/process tolerance, (b) differential
% thermal expansion between the Al7075 disc and steel pins at operating
% temperature, (c) a small running clearance for lubrication.

% --- (a) CNC process tolerance ---
% Typical 3-axis milling with good fixturing: +-0.015 mm on profile.
% Wire-EDM (recommended for thin cycloid lobes in hard/tough alloys):
% +-0.005 mm, much tighter - use if available.
cnc_tol = 0.015;      % mm, set to 0.005 if using wire-EDM

% --- (b) Differential thermal expansion, Al7075 disc vs steel pins ---
alpha_Al7075 = 23.6e-6;   % 1/degC, linear CTE of Al 7075
alpha_steel  = 12.0e-6;   % 1/degC, linear CTE of typical steel dowel pin
delta_T      = 40;        % degC, assumed max temp rise above assembly temp
                           % (motor + gearbox running heat - adjust to your
                           % measured/expected worst case)

% Radial growth of the pin-ring radius on the aluminum HOUSING side
% (if housing/ring is also Al7075) vs the steel pin diameter growth:
dR_p_thermal = R_p * (alpha_Al7075 - 0) * delta_T;   % housing ring (Al) growth
dR_r_thermal = R_r * (alpha_steel)      * delta_T;    % steel pin growth

fprintf('--- Thermal Expansion Check (Al7075 housing/disc vs steel pins) ---\n');
fprintf('Assumed delta_T above assembly temp: %.1f degC\n', delta_T);
fprintf('Pin ring radius growth (Al housing):  +%.4f mm\n', dR_p_thermal);
fprintf('Pin radius growth (steel pin):        +%.4f mm\n', dR_r_thermal);
fprintf('Net effect on radial clearance:       %.4f mm (Al grows faster -> clearance INCREASES)\n', ...
        dR_p_thermal - dR_r_thermal);
fprintf('---------------------------------------------------------------\n');

% Since Al expands ~2x faster than steel, clearance tends to OPEN UP with
% heat (opposite problem from PET-CF, which tends to shrink/close up).
% This means: size for a snug fit at ASSEMBLY (room) temperature, and
% expect slightly more backlash at operating temperature. Do NOT add a
% large negative offset here as was done for PET-CF shrinkage - a small
% negative allowance only for the CNC tolerance/lubrication film is enough.

delta_R_p = -cnc_tol;   % mm, CNC tolerance allowance only (no shrinkage term)
delta_R_r = -0.02;      % mm, running/lubrication clearance (kept from before,
                         % appropriate regardless of disc material)

R_p_mod = R_p + delta_R_p;
R_r_mod = R_r + delta_R_r;

% --- W-Output parameters (pin & hole zero-backlash output mechanism) ---
Zw          = 4;      % Number of output holes/pins (independent of N, Zp)
D_pin_out   = 6;      % mm, output steel shaft/pin diameter
use_bushing = true;   % true if a rolling bushing sleeve is used on the pin
D_hole      = D_pin_out + 2*e;   % EXACT relation for zero theoretical backlash
R_w         = 9.0;   % mm
margin_min  = 3.5;    % mm

fprintf('--- Cycloid Gear Design Parameters ---\n');
fprintf('Lobes (N): %d | Housing Pins (Zp): %d | Actual ratio: %d:1\n', N, Zp, N);
fprintf('Modified Pin Ring Radius (Rp_mod): %.3f mm\n', R_p_mod);
fprintf('Modified Housing Pin Radius (Rr_mod): %.3f mm\n', R_r_mod);
fprintf('Eccentricity (e): %.3f mm\n', e);
K1 = e*Zp/R_p_mod;
K2 = R_r_mod*Zp/R_p_mod;
fprintf('K1 (short-amplitude factor): %.3f  [target 0.4-0.75]\n', K1);
fprintf('K2 (pin size factor): %.3f  [target < 1.0]\n', K2);
fprintf('--------------------------------------\n');

% Limit Check to avoid self-intersection (cusps) in cycloid profile
if e * Zp >= R_p_mod
    error('Design Limit Violated: e*Zp = %.2f >= Rp_mod = %.2f. Profile will self-intersect!', e*Zp, R_p_mod);
end
if K2 >= 1.0
    warning('K2 = %.3f >= 1.0: high risk of profile undercut/cusping. Consider reducing R_r or increasing e.', K2);
end

% --- Material/wear warning for bare Al7075-on-steel contact ---
fprintf(['\n[MATERIAL NOTE] Al7075 disc sliding/rolling directly against bare\n' ...
         'steel pins has poor galling/wear resistance compared to PET-CF or\n' ...
         'hardened-steel-on-steel. Strongly recommend ONE of:\n' ...
         '  (1) Hard-anodize the Al7075 disc lobe surfaces (Type III / hardcoat,\n' ...
         '      ~60-70 HRC-equivalent surface, ~0.05-0.08mm thickness - adjust\n' ...
         '      R_r_mod/profile offset to compensate for coating thickness), or\n' ...
         '  (2) Use rotating steel/bronze bushings on every housing pin so the\n' ...
         '      disc contacts a hardened bushing OD, not the pin directly, or\n' ...
         '  (3) Both, for best durability under sustained peak torque loading.\n\n']);

%% 2. Profile Generation (Local Coordinate System Centered at 0,0)
% Corrected epitrochoid equations (Malhotra & Parameswaran, 1983 convention)
phi = linspace(0, 2*pi, num_points + 1);

% --- DISC 1 CALCULATION ---
psi = atan2(-sin(N*phi), (R_p_mod/(e*Zp)) - cos(N*phi));

x1 = R_p_mod*cos(phi) - e*cos(Zp*phi) - R_r_mod*cos(phi + psi);
y1 = R_p_mod*sin(phi) - e*sin(Zp*phi) - R_r_mod*sin(phi + psi);
z1 = zeros(size(phi));

% --- DISC 2 CALCULATION (180-degree phase shift disc, rotated pi/N) ---
theta_rot = pi / N;
x2 = x1 * cos(theta_rot) - y1 * sin(theta_rot);
y2 = x1 * sin(theta_rot) + y1 * cos(theta_rot);
z2 = zeros(size(phi));

%% 3. W-Output Holes (zero-backlash pin & hole output mechanism)
r_profile = sqrt(x1.^2 + y1.^2);
rho_min   = min(r_profile);              % smallest radius reached by the lobe roots

R_w_max_allowed = rho_min - D_hole/2 - margin_min;

fprintf('\n--- W-Output Hole Check ---\n');
fprintf('Number of output holes (Zw): %d\n', Zw);
fprintf('Output pin/shaft diameter: %.2f mm\n', D_pin_out);
fprintf('Required hole diameter (D_pin + 2e): %.3f mm\n', D_hole);
fprintf('Output hole circle radius (R_w): %.3f mm   <-- USE THIS VALUE IN SOLIDWORKS\n', R_w);
fprintf('Smallest profile radius (rho_min): %.3f mm\n', rho_min);
fprintf('Max allowed R_w (with %.1fmm wall margin): %.3f mm\n', margin_min, R_w_max_allowed);

if R_w > R_w_max_allowed
    warning(['R_w = %.2f mm is TOO LARGE -> output holes will cut into the cycloid ' ...
             'lobe root! Reduce R_w, reduce D_pin_out, or reduce e.'], R_w);
else
    fprintf('R_w = %.2f mm is within the safe zone.\n', R_w);
end

w_angles   = (0:Zw-1) * (2*pi/Zw);
w_holes_x  = R_w * cos(w_angles);   % hole centers on Disc 1 (local frame)
w_holes_y  = R_w * sin(w_angles);

fprintf('\nW-output hole centers (Disc 1 local/body frame, mm):\n');
fprintf('  Hole #  |  Angle (deg)  |     X       |     Y     |  Hole Dia\n');
for i = 1:Zw
    fprintf('  %2d      |    %6.1f     |  %8.3f  |  %8.3f |   %.3f\n', ...
            i, rad2deg(w_angles(i)), w_holes_x(i), w_holes_y(i), D_hole);
end

%% 4. Export Data for SolidWorks
disc1_data = [x1', y1', z1'];
file_out = 'Cycloid_Al7075_25to1.txt';
fid = fopen(file_out, 'w');
for i = 1:length(phi)
    fprintf(fid, '%.6f\t%.6f\t%.6f\r\n', disc1_data(i,1), disc1_data(i,2), disc1_data(i,3));
end
fclose(fid);

fid2 = fopen('DuBotics_Woutput_Holes_Al7075_25to1.txt', 'w');
for i = 1:Zw
    fprintf(fid2, '%.6f\t%.6f\t%.6f\r\n', w_holes_x(i), w_holes_y(i), 0);
end
fclose(fid2);
fprintf('\n(Reminder) W-output hole diameter to use in SolidWorks: %.3f mm\n', D_hole);

fprintf('\nExported coordinates successfully:\n');
fprintf(' -> Disc 1 profile: %s\n', fullfile(pwd, file_out));
fprintf(' -> W-output holes: %s\n', fullfile(pwd, 'DuBotics_Woutput_Holes_Al7075_25to1.txt'));

%% 5. Graphical Verification Plot (Assembled State View)
figure('Color', [1 1 1], 'Position', [100 100 900 900]);
hold on; grid on; box on;

x1_assembled = x1 + e;
y1_assembled = y1;
x2_assembled = x2 - e;
y2_assembled = y2;

plot(x1_assembled, y1_assembled, 'b-', 'LineWidth', 2, 'DisplayName', 'Disc 1 (Center at [+e, 0])');
plot(x2_assembled, y2_assembled, 'm-', 'LineWidth', 2, 'DisplayName', 'Disc 2 (Center at [-e, 0], Rotated \pi/N)');

pin_angles = (0:N) * (2*pi / Zp);
pin_centers_x = R_p_mod * cos(pin_angles);
pin_centers_y = R_p_mod * sin(pin_angles);

for i = 1:length(pin_angles)
    rectangle('Position', [pin_centers_x(i)-R_r_mod, pin_centers_y(i)-R_r_mod, 2*R_r_mod, 2*R_r_mod], ...
              'Curvature', [1 1], 'EdgeColor', [0.8 0.2 0.2], 'LineStyle', '-', ...
              'LineWidth', 1.2, 'HandleVisibility', 'off');
end
plot(NaN, NaN, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r', ...
     'DisplayName', sprintf('Housing Pins (%d x phi%.1fmm)', Zp, 2*R_r));

for i = 1:Zw
    hx = w_holes_x(i) + e;
    hy = w_holes_y(i);
    rectangle('Position', [hx-D_hole/2, hy-D_hole/2, D_hole, D_hole], ...
              'Curvature', [1 1], 'EdgeColor', [0.1 0.6 0.1], 'LineStyle', '-', ...
              'LineWidth', 1.2, 'HandleVisibility', 'off');
end
plot(NaN, NaN, 'gs', 'MarkerSize', 8, 'MarkerFaceColor', 'g', ...
     'DisplayName', sprintf('W-Output Holes (%d x phi%.2fmm)', Zw, D_hole));

plot(e, 0, 'kx', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Disc 1 Center');
plot(-e, 0, 'mx', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Disc 2 Center');
plot(0, 0, 'g+', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Housing/Output Center');

title(sprintf('Cycloidal Profile %d:1 - Al7075 CNC - Assembled State (with W-Output Holes)', N));
xlabel('X Coordinate (mm)');
ylabel('Y Coordinate (mm)');
axis equal;
xlim([-(R_p_mod + 2*R_r_mod + 1), R_p_mod + 2*R_r_mod + 1]);
ylim([-(R_p_mod + 2*R_r_mod + 1), R_p_mod + 2*R_r_mod + 1]);
legend('Location', 'northeastoutside');

saveas(gcf, 'cycloid_double_disc_plot_Al7075.png');
fprintf('\nSaved verification plot to: %s\n', fullfile(pwd, 'cycloid_double_disc_plot_Al7075.png'));

%% 6. Automatic Tangency Check
fprintf('\n--- Tangency Verification (theta_disc = -theta_in/N) ---\n');
theta_in_list = linspace(0, 2*pi, 12);
tol = 1e-3; % mm, acceptable deviation
all_pass = true;

for t = 1:length(theta_in_list)
    theta_in = theta_in_list(t);
    theta_disc = -theta_in / N;

    c = cos(theta_disc); s = sin(theta_disc);
    disc_x = c*x1 - s*y1 + e*cos(theta_in);
    disc_y = s*x1 + c*y1 + e*sin(theta_in);

    max_err = 0;
    for k = 1:length(pin_angles)
        dist = sqrt((disc_x - pin_centers_x(k)).^2 + (disc_y - pin_centers_y(k)).^2);
        err = abs(min(dist) - R_r_mod);
        max_err = max(max_err, err);
    end

    status = 'PASS';
    if max_err > tol
        status = 'FAIL';
        all_pass = false;
    end
    fprintf('theta_in = %.3f rad | max deviation from R_r_mod (all %d pins) = %.6f mm | %s\n', ...
            theta_in, Zp, max_err, status);
end

if all_pass
    fprintf('\n>> Profile PASSES tangency check within %.4f mm tolerance at every pin, every angle.\n', tol);
else
    fprintf('\n>> Profile FAILED tangency check. Review the epitrochoid equations.\n');
end