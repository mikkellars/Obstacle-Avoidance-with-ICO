clear all
close all

% sample_rate = 50000;
% 
% % convert Lei's analog I and C filters to digital I and C filters at 50 KHz sampling rate
% [bz_I,az_I]=impinvar([-62312.8,0],[1,8769.08,4.26601e8,1.78616e12,2.91404e16],sample_rate);
% [bz_C,az_C]=impinvar([0.000560749,2.45862,114218,0],[1,8769.08,4.26601e8,1.78616e12,2.91404e16],sample_rate);
% 
% % Generate freq. response data
% freq_cnt = 1;
% angle_cnt = 1;
%  
% start_freq = 500;
% end_freq = 2200;
% freq_steps = 50;
% 
% start_incidence_angle = deg2rad(-90);
% end_incidence_angle = deg2rad(90);
% angle_steps = deg2rad(5);
% 
% n_samples = 500;
% n_op_samples = 50;
% 
% t = 0:1/sample_rate:n_samples/sample_rate;
% 
% phase_shift_L = 0.0;
% 
% for freq = start_freq:freq_steps:end_freq
%     for angle = start_incidence_angle:angle_steps:end_incidence_angle
%         
%         phase_shift_R = 2*pi*freq*13*sin(angle)/340000;
%         
%         sinewave_L = awgn(sin(2*pi*freq*t - phase_shift_L),20);
%         sinewave_R = awgn(sin(2*pi*freq*t - phase_shift_R),20);
% %         sinewave_L = sin(2*pi*freq*t + phase_shift_L);
% %         sinewave_R = sin(2*pi*freq*t + phase_shift_R);
% 
%         Cfilter_output_L = filter(bz_C,az_C,sinewave_L); % Calculate Contralateral filter output for left sine wave
%         Ifilter_output_L = filter(bz_I,az_I,sinewave_L); % Calculate Ipsilateral filter output for left sine wave
%         Cfilter_output_R = filter(bz_C,az_C,sinewave_R); % Calculate Contralateral filter output for right sine wave
%         Ifilter_output_R = filter(bz_I,az_I,sinewave_R); % Calculate Ipsilateral filter output for right sine wave
%         
%         outL(freq_cnt,angle_cnt) = 20*log10(sum(abs(Ifilter_output_L(n_samples-n_op_samples+1:end) + ...
%                                                     Cfilter_output_R(1,n_samples-n_op_samples+1:end))));
%         outR(freq_cnt,angle_cnt) = 20*log10(sum(abs(Ifilter_output_R(1,n_samples-n_op_samples+1:end) + ...
%                                                     Cfilter_output_L(1,n_samples-n_op_samples+1:end))));
%                                                
%         % Calculate power difference of left and right side outputs
%         diff_LR(freq_cnt,angle_cnt) = outL(freq_cnt,angle_cnt) - ...
%                                       outR(freq_cnt,angle_cnt);
%        
%         angle_cnt = angle_cnt + 1;
%     end
%     freq_cnt = freq_cnt + 1;
%     angle_cnt = 1;
% end

% h = figure;
% surf(rad2deg(start_incidence_angle:angle_steps:end_incidence_angle),start_freq:freq_steps:end_freq,diff_LR);
% xlabel 'Sound Incidence Angle (deg)'; ylabel 'Sound Frequency (Hz)'; zlabel 'Vibration amplitude difference (dB)';
% title 'Ear model response';
% axis square;

grey_blue_gradient = 0.01*[70, 70, 70;
68, 68, 70;
67, 67, 71;
65, 65, 71;
64, 64, 72;
62, 62, 73;
61, 61, 73;
60, 60, 74;
58, 58, 75;
57, 57, 75;
55, 55, 76;
54, 54, 76;
53, 53, 77;
51, 51, 78;
50, 50, 78;
48, 48, 79;
47, 47, 80;
45, 45, 80;
44, 44, 81;
42, 42, 81;
41, 41, 82;
40, 40, 83;
38, 38, 83;
37, 37, 84;
35, 35, 84;
34, 34, 85;
33, 33, 85;
31, 31, 86;
30, 30, 87;
28, 28, 87;
27, 27, 88;
25, 25, 89;
24, 24, 89;
23, 23, 90;
21, 21, 91;
20, 20, 91;
18, 18, 92;
17, 17, 93;
15, 15, 93;
14, 14, 94;
13, 13, 94;
11, 11, 95;
10, 10, 96;
8, 8, 96;
7, 7, 97;
5, 5, 97;
4, 4, 98;
3, 3, 98;
1, 1, 99;
0, 0, 100];
%       r g b
colr = [0.6 0.6 0.6;
        1 0.7 0;
        0 0 1;
        0 0 0;
        1 0 0;
        0.7 0 0.7;
        0 0.7 0
        ];

sample_rate = 50000;
% convert Lei's analog I and C filters to digital I and C filters at 50 KHz sampling rate
[bz_I,az_I]=impinvar([-62312.8,0],[1,8769.08,4.26601e8,1.78616e12,2.91404e16],sample_rate);
[bz_C,az_C]=impinvar([0.000560749,2.45862,114218,0],[1,8769.08,4.26601e8,1.78616e12,2.91404e16],sample_rate);
n_samples = 500;
n_op_samples = 50;
t = 0:1/sample_rate:n_samples/sample_rate;
freq = 2200;
spkr_angle = deg2rad(50);
r = 300; % straight line distance between robot and goal in cm
v_l = 0; % linear velocity of left wheel cm/s
v_r = 0; % linear velocity of right wheel cm/s
l = 16; % width of robot in cm
sensor_threshold = 20; % distance sensor threshold below which the reflex
                       % triggered

sensor_threshold2 = 40; % distance sensor threshold below which an obstacle
                       % is detected
                       
snr_db = 20; % signal to noise ratio in dB for gaussian noise model

add_noise = 1;
intermittent_sound = 0;
off_time = 0;
off_t = 0;
rng('shuffle');
n = round(2+10*rand);

spkr_xy = [r*cos(spkr_angle) r*sin(spkr_angle)]; % sound source position as
                                                 % [x y]

robot_pose = [0; 0; deg2rad(0)]; % robot pose [x y theta]
robot_omega = (v_r - v_l)/l; % robot angular velocity
sensor = [robot_pose(1) robot_pose(2) ...
          sensor_threshold sensor_threshold2]; % distance sensor
                                               % position and
                                               % thresholds
                                               % [x y t t2]

n_obst = 15; % no. of obstacles
obst = [round((1.75*r-0.7*r)*rand(n_obst,2)) ...
        round(5+((34000/freq)-5)*rand(n_obst,1))]; % obstacle location and radius
                                         % [x y r]. Radius is between 5 and
                                         % 34000/freq = sound wavelength.
for i = 1:n_obst
    if (sqrt((spkr_xy(1)-obst(i,1))^2 + (spkr_xy(2)-obst(i,2))^2) < sensor_threshold)
        obst(i,1:2) = [obst(i,1) obst(i,2)]+1.5*sensor_threshold;
    end
end

% obst = [309    42    12; % Fig. 1, NCA paper
%    215   183    13;
%     41    98    14;
%    164   102     6;
%    158   107    11;
%     63    99    13;
%     95    27    15;
%    206    21     7;
%    233   147     8;
%     64    54    11;
%     16   264     6;
%    213   269    11;
%    163   265    10;
%    112   249     7;
%     71    81    13];

% obst = [256   245     6; % Fig. 2, NCA paper
%    239   161     6;
%     14    31     6;
%    236   291    15;
%    220    24    10;
%    233   197    10;
%    308   289    15;
%    144   193     6;
%    269   107    14;
%    115   293    11;
%    142   125     5;
%    210   180    10;
%    306   155    15;
%    142   250    12;
%     95   314    15];

% obst = [117   294    15; % ** Fig. 3, NCA paper
%    161   306     7;
%    153   121    10;
%    209   188    12;
%    271    79    13;
%    108   250    13;
%    296   181     6;
%    239     4    13;
%    258   254     7;
%    234   123    14;
%     59   166    13;
%     77   131    11;
%    304    83     9;
%    178   186    15;
%    244   260     9];

% obst = [219   170    10; % ** Fig. 6, NCA paper
%    173    89    15;
%     39    62     6;
%    275   161     9;
%    100    13    11;
%    145   113    12;
%    293   149    15;
%    312   274     7;
%    200    30    14;
%    202   198     9;
%    306   217    11;
%    110   202    13;
%    274   170    15;
%     54   181    14;
%     63   177    14];

% obst = [55   280    12; % ** % Fig. 7, NCA paper
%    152   204     6;
%    176    59    12;
%    287   241    13;
%    228   102    13;
%    300   126     8;
%     91   206     6;
%    242   182     8;
%     59   219    10;
%    229    76    15;
%    251   158     8;
%    163   112    15;
%      8    29     8;
%    100   192     6;
%    122    17     9];

% obst = [4   300     8; % no
%    210   177     9;
%    250    58    14;
%     92   134    11;
%    246   313    13;
%    171   258    12;
%    189    92    13;
%     75   122    14;
%     18    86     8;
%    218   161    14;
%    185    86     5;
%    274   169     8;
%    189   100     7;
%    142   236    10;
%    137   277     5];

% obst = [307    94     8; % *** % Fig 5., NCA paper
%     88    73     9;
%    165   234    10;
%    249   250    13;
%     42    63    11;
%     83   261    12;
%     68   274     7;
%    186     9    15;
%    216    23     6;
%    315   227     6;
%     64   223     9;
%    301    21     9;
%    167   192    13;
%     42   156    14;
%    191   136    14];

% obst = [200    96     6; % good. works. Clear diff. between learn and
%    285    13    10;       % no learn *** Fig. 4, NCA paper
%     32   278     7;
%     48    51    11;
%     57   195    15;
%     26    31     8;
%      9    81     7;
%    308    55     9;
%    176   187    15;
%    302   298    15;
%    278   196    15;
%    122   112     8;
%     34   196    15;
%    221   258    15;
%    120    81     6];

% obst = [   285   133     5; % simple, works with or without 2nd ICO learner
%     40   288     8;
%    288   250     5;
%    199   302     6;
%     31   207    14;
%     88    11    12;
%    172   267     8;
%    302   294    15;
%    304   214     5;
%     50   239    10;
%    306   234     9;
%    302   124    13;
%    153   206    13;
%    252    54     7;
%     45   222    10];

% obst = [   119   242    14; % good. works. Clear diff. between learn and
%    214    46    13;         % no learn ** % Fig. 8, NCA paper
%    259   146     9;
%    150   172    11;
%    273   126     8;
%     89   314    15;
%     16   182    11;
%    265   288    11;
%    124   141    12;
%    293   145     5;
%      4   271    14;
%    261   245     9;
%    259   156    10;
%     88   118     8;
%     90    43     8];

% obst = [39   145     9; % no
%    240    68     8;
%     53    45     6;
%    283   179     8;
%    144   172    12;
%    304   144    14;
%    311   172    13;
%     86   244    11;
%     14   134     9;
%    224    18    14;
%    220    39     5;
%    193   190    14;
%    270    35     7;
%    269   259     7;
%    169   274     8];

% obst = [273   133     7; % no
%    195   296     6;      
%    136   136    15;
%     50    40     8;
%     38   262     7;
%    195   311    14;
%    244   231    13;
%    314    38    15;
%    269   156    13;
%    207    24    10;
%      2   313    14;
%     98    21    14;
%    104   295    14;
%     89   238     5;
%    146   199    15];

% obst = [221    61     8; % Less simple. Works.
%    121    82    12;      % **
%     49    74     7;
%    103   312     8;
%    137   131    14;
%    237   148    15;
%     55   260    15;
%     25   271     8;
%     14   166     9;
%     12   228    10;
%    110   198     6;
%     31    52     7;
%     51   152     6;
%    175   312    10;
%    182   134    13];

% obst = [104   116    14; % Slightly hard, works. Clear diff. between
%    271   144     6;      % no-learning and learnin
%     58   302    10;      % ** % Fig. 9, NCA paper
%    234   136    14;
%    166   154     9;
%    312   121    15;
%    127   137     7;
%    220   306     7;
%    238   277     6;
%    281   305    11;
%    127     9    11;
%     26   254     9;
%    246   303    10;
%    179    50    10;
%    158   216     5];

% obst = [    84   162    15; % Slightly hard, works. Clear diff. between
%    378   104    14;         % no-learning and learnin
%     29    55     6;         % ** % Fig. 10, NCA paper
%    174    91    13;
%    218   182     6;
%    364    26    14;
%    323    84     6;
%    184   112     9;
%    196    16    14;
%    412   271     8;
%     29    33    14;
%    289   145    10;
%    239    69     9;
%    174   130    10;
%    338   351    15];

% obst = [436   417     6;    % Hard. Stable. Works. Robust to obstacles near
%     85   225     9;         % goal.
%    223   435     7;
%    214   387    11;
%    358   141    14;
%    238   227     8;
%    132   387    14;
%    214   245    15;
%    111   322    15;
%     86    19    10;
%    172   198    14;
%     30   243    12;
%     34   309    14;
%    370   273    10;
%    267   154    12];
 
% obst = [    62    17    13; % Less simple. Works.
%    205   109     7;         % *
%    439   117     7;
%    402   157    13;
%    130   390    15;
%    115   325    14;
%    283   431     9;
%     89   231    15;
%     65    45     5;
%    143   412    15;
%    285   345    14;
%    402    17    14;
%    382   193     9;
%    287   327    10;
%    160   230     8];

% obst = [    87   229    15; % simple, works but almost no difference
%    352   323    10;         % between no-learning and learning
%    228   392     6;
%    304   188    13;
%     65    26    13;
%    149    48     9;
%    414    59    10;
%    403   323    14;
%     89   323    12;
%    218   169    12;
%    274   105     5;
%    233   380     7;
%    426    99    13;
%    132    50    12;
%    115   330     9];

% obst = [   186     2     5; % simple, works but not so much difference
%    344    69     6;         % between no-learning and learning
%    104    12    14;         % *
%    194   335    14;
%    416   148     6;
%     53    32    13;
%    295   304    11;
%     64    93     9;
%    329   365     6;
%    282   366    11;
%     26   355     7;
%     43   233    15;
%    195   276     7;
%    217   244    10;
%    131   431    13];

obst = [124   199     8; % ** % Fig. 11, NCA paper
   193    28     6;
     9   290    12;
   247     5    15;
    41    40    10;
     9    86     5;
   227   165    15;
   242    42    14;
    97   174    10;
    52   126     7;
    97    66    14;
   233   208     9;
    36   164    10;
   302   170    14;
   210    19    11];

% obst = [1    81    10; % Wall 1
%         21    81    10;
%         41    81    10;
%         61    81    10;
%         81    81    10;
%         101    81    10;
%         121    81    10;
%         121    101   10;
%         221    141   10;
%         221    121   10;
%         141    161    10
%         161    161    10;
%         181    161    10;
%         201    161    10;
%         221    161    10];
    
% obst = [280    280    10; % Room
%         260    280    10;
%         240    280    10;
%         220    280    10;
%         200    280    10;
%         180    280    10;
%         160    280    10;
%         140    280    10;
%         280    140    10;
%         260    140    10;
%         240    140    10;
%         220    140    10;
%         200    140    10;
%         180    140    10;
%         160    140    10;
%         140    140    10;
%         140    160    10;
%         140    180    10;
%         140    200    10;
%         140    220    10;
%         140    240    10;
%         140    260    10;
%         ];

% obst = [280    280    10; % Room 2
%         260    280    10;
%         240    280    10;
%         220    280    10;
%         200    280    10;
%         180    280    10;
%         160    280    10;
%         140    280    10;
%         280    140    10;
%         260    140    10;
%         240    140    10;
%         220    140    10;
%         200    140    10;
%         180    140    10;
%         160    140    10;
%         140    140    10;
%         140    160    10;
%         140    180    10;
%         140    200    10;
%         140    220    10;
%         140    240    10;
%         140    260    10;
%         280    160    10;
%         280    180    10;
%         280    200    10;
%         ];

% obst = [280    280    10; % Room 2
%         260    280    10;
%         240    280    10;
%         220    280    10;
%         200    280    10;
%         180    280    10;
%         160    280    10;
%         140    280    10;
%         280    140    10;
%         260    140    10;
%         240    140    10;
%         220    140    10;
%         200    140    10;
%         180    140    10;
%         160    140    10;
%         140    140    10;
%         140    160    10;
%         140    180    10;
%         140    200    10;
%         140    220    10;
%         140    240    10;
%         140    260    10;
%         280    160    10;
%         280    180    10;
%         280    200    10;
%         100     80    10;
%         100     60    10;
%         100     40    10;
%         100     20    10;
%         100      0    10;
%         100    -20    10;
%         100    -40    10;
%         100    -60    10;
%         100     80    10;
%         20      80    10;
%         0       80    10;
%         -20     80    10;
%         -40     80    10;
%         ];

% obst = [0    100    10;
%         20   100    10;
%         40   100    10;
%         60   100    10;
%         80   100    10;
%         160   140    10;
%         180   140    10;
%         200   140    10;
%         200   160    10;
%         ]; % Wall 3
n_obst = size(obst,1);
    
omega = 2*pi*freq;
c = omega*13/340000;

w_bl = 0.1*rand; % ICO synaptic weight for left interneuron
w_br = 0.1*rand; % ICO synaptic weight for right interneuron
w = 0; % ICO synaptic weight for obstacle avoidance learner
eta1 = 0.01; % ICO learning rate for Braitenberg couplings
eta2 = 0.2;% ICO learning rate for path smoothing
bl = rand; % activation function parameter for left interneuron
br = rand; % activation function parameter for right interneuron
b = 1;
w_test = 0.5;
b_test = rand;
% set up plots
subfigs_on = 0;

scrsz = get(groot,'ScreenSize');
fh = figure('Position', ...
       [scrsz(1) scrsz(2) scrsz(3) scrsz(4)]); % [left bottom width height]

% plot initial interneuron activation functions
h3 = subplot(4,2,2);
z = -10:0.1:10;
% plot(h3,z,4./(1+b_test*exp(-b*z)),'.b','LineWidth',2); % right
plot(h3,z,4./(1+br*exp(-b*z)),'.b','LineWidth',2); % right
grid on;
hold on;
% plot(h3,z,4./(1+b_test*exp(-b*z)),'.r','LineWidth',2); % left
plot(h3,z,4./(1+bl*exp(-b*z)),'.r','LineWidth',2); % left
legh = legend('Right interneuron','Left interneuron');
legh.Location = 'northwest';
% title 'Intrinsic excitability (neuron activation function)';
% empty plot for ICO inputs
h2 = subplot(4,2,[6 8]);
h2.XLim = [1 1000];
h2.YLim = [-3 10];
grid on;
hold on;
h4 = subplot(4,2,4);
h4.XLim = [1 1000];
h4.YLim = [0 8];
grid on;
hold on;
% plot with sound source, initial robot pose and obstacles
h1 = subplot(4,2,[1 3 5 7]);
h1.XLim = [-r/2 r*2];
h1.YLim = [-r/2 r*2];
box on;
grid on;
axis square;
hold on;
plot(h1,spkr_xy(1),spkr_xy(2),'sk','MarkerFaceColor','black'); % sound source
for i = 1:n_obst % obstacles
    h(i) = viscircles(obst(i,1:2),obst(i,3));
    x = cell2mat(get(h(i).Children,'XData'));
    y = cell2mat(get(h(i).Children,'YData'));
    hf(i) = fill(x(2,1:end-1),y(2,1:end-1),'r');
end
% theta = zeros(1);
plot(h1,robot_pose(1),robot_pose(2),'ok'); % initial robot location
line([robot_pose(1) robot_pose(1)+5*cos(robot_pose(3))],...
     [robot_pose(2) robot_pose(2)+5*sin(robot_pose(3))]); % initial robot
                                                          % heading
rl = 0;
rr = 0;
itr = 0;

record_avi = 0;
% frames = 0;

if (record_avi == 1)
    writerObj = VideoWriter('path_smoothing_only_fig2.mp4','MPEG-4');
    %     writerObj = VideoWriter('1nav_during_learning.mp4','MPEG-4');
    open(writerObj);
%     loops = size(1:50000);
%     F(loops(1,2)) = struct('cdata',[],'colormap',[]);
end

% bl = 3.5;
% br = bl;

% start learning
for itr = 1:50 % learning iterations
    % reset 
    robot_pose = [0; 0; deg2rad(0)];
    robot_omega = (v_r - v_l)/l;
    sensor = [robot_pose(1) robot_pose(2) ...
              sensor_threshold sensor_threshold2];
    reflex = 0;
    dist_moved(itr) = 0;
    diff = [];
    d_diff = [];
    dist_min = [];
    oL = [];
    oR = [];
    vl = [];
    vr = [];
    wbl = [];
    wbr = [];
    wdist = [];
    
    off_time = 0;
    off_t = 0;
    rng('shuffle');
    n = round(2+10*rand);
    
%     if bl == 0.5
%         bl = 30.97;
%         br = 3.76;
%     else
%         bl = bl - 0.5;
%         br = bl;
%     end
%     cla(h3);
%     if (isobject('a6'))
%         delete(a6)
%     end
%     a6 = plot(h3,z,4./(1+br*exp(-b*z)),'.b','LineWidth',2); % right
%     grid on;
%     hold on;
%     if (isobject('a7'))
%         delete(a7)
%     end
    % plot(h3,z,4./(1+b_test*exp(-b*z)),'.r','LineWidth',2); % left
%     a7 = plot(h3,z,4./(1+bl*exp(-b*z)),'.r','LineWidth',2); % left
%     legh = legend('Right interneuron','Left interneuron');
%     legh.Location = 'northwest';
for ts = 1:1000 % time steps in seconds
    
    last_robot_xy = [robot_pose(1) robot_pose(2)]; % last robot position
                                                   % used to calculate
                                                   % distance travelled

    % Determine speaker position relative to the robot
    spkr_angle = atan2(spkr_xy(2)-robot_pose(2),spkr_xy(1)-robot_pose(1));

    % Sense the sound direction with the ear model
    phase_diff = c*sin(robot_pose(3)-spkr_angle);
    
    if add_noise == 0
        sinewave_L = sin(omega*t - 0);
        sinewave_R = sin(omega*t - phase_diff);
    else
        sinewave_L = awgn(sin(omega*t - 0),snr_db);
        sinewave_R = awgn(sin(omega*t - phase_diff),snr_db);
    end

    Cfilter_output_L = filter(bz_C,az_C,sinewave_L);
    Ifilter_output_L = filter(bz_I,az_I,sinewave_L);
    Cfilter_output_R = filter(bz_C,az_C,sinewave_R);
    Ifilter_output_R = filter(bz_I,az_I,sinewave_R);

    outL = 20*log10(sum(abs(Ifilter_output_L(n_samples-n_op_samples+1:end) + ...
                            Cfilter_output_R(1,n_samples-n_op_samples+1:end))));
    outR = 20*log10(sum(abs(Ifilter_output_R(1,n_samples-n_op_samples+1:end) + ...
                            Cfilter_output_L(1,n_samples-n_op_samples+1:end))));
    
    if (intermittent_sound == 1)
        if (off_time > 0)
            off_time = off_time - 1;
%             oL(ts) = 0;
%             oR(ts) = 0;
            oL(ts) = oL(ts-1);
            oR(ts) = oR(ts-1);
        else
            if (mod(ts,n) == 0)
                n = round(30+(15-10)*rand);
                off_time = floor(1 + (10-0).*rand);
%                 oL(ts) = 0;
%                 oR(ts) = 0;
                oL(ts) = oL(ts-1);
                oR(ts) = oR(ts-1);
                if exist('htxt8')
                    delete(htxt8); 
                end
                htxt8 = text(300,600-125,'Sound off');
            else
                oL(ts) = 118-abs(outL);
                oR(ts) = 118-abs(outR);
                if exist('htxt8')
                    delete(htxt8); 
                end
                htxt8 = text(300,600-125,'Sound on');
                off_time = 0;
            end
        end
    else
        oL(ts) = 118-abs(outL);
        oR(ts) = 118-abs(outR);
        off_time = 0;
    end
    
    % Determine the distance to the obstacles and find all obstacles within
    % distance sensor threshold (20cm)
    for i = 1:n_obst
        dist(i,1) = sqrt((sensor(2)-obst(i,2))^2 + (sensor(1)-obst(i,1))^2) ...
                    - obst(i,3);
        if add_noise == 1
            dist(i,1) = awgn(dist(i,1),3);
        end
        if (dist(i,1) <= sensor(4))
            dist(i,3) = dist(i,1);
        else
            dist(i,3) = 0;
        end
        if (dist(i,1) <= sensor(3))
            dist(i,2) = dist(i,1);
        else
            dist(i,2) = 0;
        end
    end
    
    % Determine the distance to the closest obstacle
    if min(find(dist(:,2)))
        dist_min(ts) = dist(min(find(dist(:,2))),1);
    else
        dist_min(ts) = 0;
    end
    
    % If the obstacle is closer than the distance sensor threshold, perform
    % evasive manoeuver by determining whether the obstacle is the left or
    % right (not how much to left or right) and turning with fixed
    % velocity in the opposite direction. Also, update the ICO weights
    % during reflex behaviour. If the obstacle is further away, use
    % Braitenberg steering to move towards sound source with latest
    % calculated velocity
    if min(find(dist(:,2)))
        reflex = reflex + 1;
        theta = (atan2(obst(min(find(dist(:,2))),2)-sensor(2),...
                      obst(min(find(dist(:,2))),1)-sensor(1))-...
                      robot_pose(3));
        if ((theta < 0) && (theta > deg2rad(-90))) % right side of robot
            rr = 1;
            v_l = 0.1;
            v_r = 4;
%             w_test = w_test + ...
%                    eta1*oR(ts)*(dist_min(ts)-dist_min(ts-1))/sensor(3);
%             b_test = b_test + ...
%                  w_test*oR(ts) + dist_min(ts)/sensor(3);

            w_br = w_br + ...
                   eta1*oR(ts)*(dist_min(ts)-dist_min(ts-1))/sensor(3);
            br = br + ...
                 w_br*oR(ts) + dist_min(ts)/sensor(3);
        elseif ((theta > 0)  && (theta < deg2rad(90))) % left side of robot
            rl = 1;
            v_l = 4;
            v_r = 0.1;
%             w_test = w_test + ...
%                    eta1*oR(ts)*(dist_min(ts)-dist_min(ts-1))/sensor(3);
%             b_test = b_test + ...
%                  w_test*oR(ts) + dist_min(ts)/sensor(3);

            w_bl = w_bl + ...
                   eta1*oL(ts)*(dist_min(ts)-dist_min(ts-1))/sensor(3);
            bl = bl + ...
                 w_bl*oL(ts) + dist_min(ts)/sensor(3);
        end
        w = w + ...
            eta2*dist(min(find(dist(:,2))),3)/sensor(4)*(dist_min(ts)-dist_min(ts-1))/sensor(3);
%         viscircles(h1,sensor(1:2),sensor(3),'EdgeColor','k','LineStyle','-.')
        
        wbl(ts) = w_bl;
        wbr(ts) = w_br;
        wdist(ts) = w;
        
        % update activation function plot
        if (numel(h3.Children) ~= 2)
            delete(a6);
            delete(a7);
        end
%         a6 = plot(h3,z,4./(1+b_test*exp(-b*z)),'b','LineWidth',2);
%         a7 = plot(h3,z,4./(1+b_test*exp(-b*z)),'r','LineWidth',2);
        a6 = plot(h3,z,4./(1+br*exp(-b*z)),'b','LineWidth',2);
        a7 = plot(h3,z,4./(1+bl*exp(-b*z)),'r','LineWidth',2);
    else
        % no reflex, Braitenverg cross-coupling
        if (find(dist(:,3)))
            v = w*dist(min(find(dist(:,3))),3)/sensor(4) + dist_min(ts)/sensor(3);            
            theta = (atan2(obst(min(find(dist(:,3))),2)-sensor(2),...
                      obst(min(find(dist(:,3))),1)-sensor(1))-...
                      robot_pose(3));
            if ((theta < 0) && (theta > deg2rad(-90))) % right side of robot
%                 v_l = 4/(1+b_test*exp(-b*oR(ts)));
%                 v_r = 4/(1+b_test*exp(-b*oL(ts))) + v;
                v_l = 4/(1+br*exp(-b*oR(ts)));
                v_r = 4/(1+bl*exp(-b*oL(ts))) + v;
            elseif ((theta > 0)  && (theta < deg2rad(90))) % left side of robot
%                 v_l = 4/(1+b_test*exp(-b*oR(ts))) + v;
%                 v_r = 4/(1+b_test*exp(-b*oL(ts)));
                v_l = 4/(1+br*exp(-b*oR(ts))) + v;
                v_r = 4/(1+bl*exp(-b*oL(ts)));
            else
%                 v_l = 4/(1+b_test*exp(-b*oR(ts)));
%                 v_r = 4/(1+b_test*exp(-b*oL(ts)));
                v_l = 4/(1+br*exp(-b*oR(ts)));
                v_r = 4/(1+bl*exp(-b*oL(ts)));
            end
        else
%             v_l = 4/(1+b_test*exp(-b*oR(ts)));
%             v_r = 4/(1+b_test*exp(-b*oL(ts)));
            v_l = 4/(1+br*exp(-b*oR(ts)));
            v_r = 4/(1+bl*exp(-b*oL(ts)));
        end
    end
    vl(ts) = v_l;
    vr(ts) = v_r;
%     error(ts) = deg2rad(robot_pose(3)-spkr_angle);
    
    % update robot's pose based on new motor velocities
    if (v_l ~= v_r)
        R = (l/2)*(v_r + v_l)/(v_r - v_l);
        robot_omega = (v_r - v_l)/l;
        icc = [robot_pose(1) - (R*sin(robot_pose(3))), ...
               robot_pose(2) + (R*cos(robot_pose(3)))];
        robot_pose = [cos(robot_omega) -sin(robot_omega) 0;...
                      sin(robot_omega)  cos(robot_omega) 0;...
                      0                 0                1]...
                      *...
                      [R*sin(robot_pose(3));...
                      -R*cos(robot_pose(3));...
                      robot_pose(3)]...
                      +...
                      [icc(1);...
                      icc(2);...
                      robot_omega];               
        
        if (abs(robot_pose(3)) > pi)
            robot_pose(3) = robot_pose(3) - sign(robot_pose(3))*2*pi;
        end
    else
        robot_omega = 0;
        robot_pose = robot_pose + ...
                    [cos(robot_pose(3)/v_l); ...
                     sin(robot_pose(3)/v_l); ...
                     0];
    end
    
    % update sensor location;
    sensor(1:2) = [robot_pose(1) robot_pose(2)];
    
    % Calculate distance travelled by robot
    dist_moved(itr) = dist_moved(itr) + ...
                      sqrt((robot_pose(2)-last_robot_xy(2))^2 + ...
                           (robot_pose(2)-last_robot_xy(2))^2);
    
    % update robot's pose in plot
    a1 = plot(h1,robot_pose(1),robot_pose(2),'ok');
    a2 = line([robot_pose(1),robot_pose(1)+5*cos(robot_pose(3))],...
         [robot_pose(2),robot_pose(2)+5*sin(robot_pose(3))],'Color',...
         [grey_blue_gradient(itr,1) grey_blue_gradient(itr,2) ...
         grey_blue_gradient(itr,3)]);
%     a2 = line([robot_pose(1),robot_pose(1)+5*cos(robot_pose(3))],...
%               [robot_pose(2),robot_pose(2)+5*sin(robot_pose(3))],...
%               'Color',[colr(itr,1) colr(itr,2) colr(itr,3)],'LineWidth',3);
    
    % print out motor velocities, activation function parameters,
    % iterations, stime steps and distance travelled
    htxt1 = text(robot_pose(1)-150,robot_pose(2)-25,strcat('v_l = ',num2str(v_l)));
    htxt2 = text(robot_pose(1)-150,robot_pose(2)-45,strcat('v_r = ',num2str(v_r)));
    htxt3 = text(300,600-25,strcat('bl =',num2str(bl)));
    htxt4 = text(300,600-45,strcat('br =',num2str(br)));
    htxt5 = text(300,600-65,strcat('Iteration = ',num2str(itr)));
    htxt6 = text(300,600-85,strcat('Time step = ',num2str(ts)));
    htxt7 = text(300,600-105,strcat('Distance travelled = ',num2str(dist_moved(itr))));
    
    
    % update ICO inputs in plot
    if (subfigs_on == 1)
        a3 = plot(h2,-2+dist_min/sensor(3),'g','LineWidth',2);
        a4 = plot(h2,oR,'b','LineWidth',2);
        a5 = plot(h2,oL,'r','LineWidth',2);
        a8 = plot(h4,vl,'m','LineWidth',2);
        a9 = plot(h4,vr,'k','LineWidth',2);
    end
    
    if (record_avi == 1)
%         frames = frames + 1;
%         F(frames) = getframe(fh);
          frm = getframe(fh);
          writeVideo(writerObj,frm);
    end
    
    pause(0.0001); % pause execution to create animation
    
    % update plots
    delete(htxt1);
    delete(htxt2);
    delete(htxt3);
    delete(htxt4);
    delete(htxt5);
    delete(htxt6);
    delete(htxt7);
    delete(a1);
    
    % determine distance of robot from sound source
    dist = sqrt((spkr_xy(2)-robot_pose(2))^2 + ...
                (spkr_xy(1)-robot_pose(1))^2);
            
    % exit iteration when robot is close enough to sound source
    if (dist < 10)
       break;
    end
    
%     if robot_pose(2) > spkr_xy(2) + 50
%        break; 
%     end
    
    % exponentially reduce both bl and br
    if (rl == 1)
%         b_test = b_test*exp(-ts/60000);
        bl = bl*exp(-ts/60000);
        if (subfigs_on == 1)
        % update activation function plot
        if (numel(h3.Children) ~= 2)
            delete(a7);
        end
%         a7 = plot(h3,z,4./(1+b_test*exp(-b*z)),'r','LineWidth',2);
        a7 = plot(h3,z,4./(1+bl*exp(-b*z)),'r','LineWidth',2);
        end
    end
    if (rr == 1)
%         b_test = b_test*exp(-ts/60000);
        br = br*exp(-ts/60000);
        if (subfigs_on == 1)
        % update activation function plot
        if (numel(h3.Children) ~= 2)
            delete(a6);
        end
%         a6 = plot(h3,z,4./(1+b_test*exp(-b*z)),'b','LineWidth',2);
        a6 = plot(h3,z,4./(1+br*exp(-b*z)),'b','LineWidth',2);
        end
    end
end

% exit learning only when reflex is no longer triggered and robot reaches
% sound source
if ((reflex == 0) && (dist < 10))
    break;
end

% update ICO input plot
cla(h2);
cla(h4);
end

% print out stuff
htxt3 = text(200,600-25,strcat('bl =',num2str(bl)));
htxt4 = text(200,600-45,strcat('br =',num2str(br)));
htxt5 = text(200,600-65,strcat('Iterations = ',num2str(itr)));
htxt6 = text(200,600-85,strcat('Time steps = ',num2str(ts)));
htxt7 = text(200,600-105,strcat('Distance travelled in 1st iteration = ', ...
             num2str(dist_moved(1))));
htxt8 = text(200,600-125,strcat('Distance travelled in last iteration = ', ...
             num2str(dist_moved(itr))));
% update 
% a4 = plot(h2,-2+dist_min/sensor(3),'g','LineWidth',3);
% a5 = plot(h2,d_diff,'r','LineWidth',3);

% figure;plot(error);

%save('workspace_after_learning.mat','-v7.3');

if (record_avi == 1)
    close(writerObj);
%     movie2avi(F,'nav1.avi','compression','Indeo5');
end

pause;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if (record_avi == 1)
    writerObj = VideoWriter('2nav_fixed_braitenberg.mp4','MPEG-4');
    open(writerObj);
%     loops = size(1:1000);
%     F(loops(1,2)) = struct('cdata',[],'colormap',[]);
end

cla(h1);
cla(h2);
cla(h4);
robot_pose = [0; 0; deg2rad(0)]; % [x y theta]
v_l = 0; % linear velocity of left wheel cm/s
v_r = 0; % linear velocity of right wheel cm/s
robot_omega = (v_r - v_l)/l;
sensor = [robot_pose(1) robot_pose(2) ...
          sensor_threshold sensor_threshold2]; % distance sensor
                                               % position and
                                               % thresholds
                                               % [x y t t2]
diff = [];
d_diff = [];
dist_min = [];
dist_moved(itr + 1) = 0;
oL = [];
oR = [];
vl = [];
vr = [];
% obst = [40+round((1.75*r-0.25*r)*rand(n_obst,2)) ...
%         round(5+((34000/freq)-5)*rand(n_obst,1))]; % obstacle location and radius
%                                          % [x y r]. Radius is between 5 and
%                                          % 34000/freq = sound wavelength.

plot(h1,spkr_xy(1),spkr_xy(2),'sk','MarkerFaceColor','black');
% plot(h1,obst(:,1),obst(:,2),'xr');
for i = 1:n_obst
    h(i) = viscircles(obst(i,1:2),obst(i,3));
    x = cell2mat(get(h(i).Children,'XData'));
    y = cell2mat(get(h(i).Children,'YData'));
    hf(i) = fill(x(2,1:end-1),y(2,1:end-1),'r');
end
theta = zeros(1);
plot(h1,robot_pose(1),robot_pose(2),'ok');
line([robot_pose(1),robot_pose(1)+5*cos(robot_pose(3))],...
[robot_pose(2),robot_pose(2)+5*sin(robot_pose(3))]);

learning_off = 1;
for ts = 1:1000 % in seconds
    
    last_robot_xy = [robot_pose(1) robot_pose(2)]; % last robot position
                                                   % used to calculate
                                                   % distance travelled
    
    % Determine speaker position relative to the robot
    spkr_angle = atan2(spkr_xy(2)-robot_pose(2),spkr_xy(1)-robot_pose(1));

    % Sense the sound direction with the ear model
    phase_diff = c*sin(robot_pose(3)-spkr_angle);

    if add_noise == 0
        sinewave_L = sin(omega*t - 0);
        sinewave_R = sin(omega*t - phase_diff);
    else
        sinewave_L = awgn(sin(omega*t - 0),snr_db);
        sinewave_R = awgn(sin(omega*t - phase_diff),snr_db);
    end

    Cfilter_output_L = filter(bz_C,az_C,sinewave_L);
    Ifilter_output_L = filter(bz_I,az_I,sinewave_L);
    Cfilter_output_R = filter(bz_C,az_C,sinewave_R);
    Ifilter_output_R = filter(bz_I,az_I,sinewave_R);

    outL = 20*log10(sum(abs(Ifilter_output_L(n_samples-n_op_samples+1:end) + ...
                            Cfilter_output_R(1,n_samples-n_op_samples+1:end))));
    outR = 20*log10(sum(abs(Ifilter_output_R(1,n_samples-n_op_samples+1:end) + ...
                            Cfilter_output_L(1,n_samples-n_op_samples+1:end))));

    if (intermittent_sound == 1)
        if (mod(ts,n) == 0)
            n = round(2+10*rand);
            oL(ts) = 0;
            oR(ts) = 0;
        else
            oL(ts) = 118-abs(outL);
            oR(ts) = 118-abs(outR);
        end
    else
        oL(ts) = 118-abs(outL);
        oR(ts) = 118-abs(outR);
    end
    
    % Determine the distance to the obstacles and find all obstacles within
    % distance sensor threshhold (20cm)
    for i = 1:n_obst
        dist(i,1) = sqrt((sensor(2)-obst(i,2))^2 + (sensor(1)-obst(i,1))^2) ...
                    - obst(i,3);
        if add_noise == 1
            dist(i,1) = awgn(dist(i,1),3);
        end
        if (dist(i,1) <= sensor(3))
            dist(i,2) = i;
        else
            dist(i,2) = 0;
        end
    end
    
    % Determine the distance to the closest obstacle
    if min(find(dist(:,2)))
        dist_min(ts) = dist(min(find(dist(:,2))),1);
    else
        dist_min(ts) = 0;
    end
    
    % If the obstacle is closer than the distance sensor threshold, perform
    % evasive manoeuver by determining whether the obstacle is the left or
    % right (not how much to left or right) and turning with fixed
    % velocity in the opposite direction.
    if min(find(dist(:,2)))
        theta = (atan2(obst(min(find(dist(:,2))),2)-sensor(2),...
                      obst(min(find(dist(:,2))),1)-sensor(1))-...
                      robot_pose(3));
        if ((theta < 0) && (theta > deg2rad(-90))) % right side of robot
            v_l = 0.1;
            v_r = 4;
        elseif ((theta > 0)  && (theta < deg2rad(90))) % left side of robot
            v_l = 4;
            v_r = 0.1;
        end
    else
        if (learning_off == 0)
        % no reflex, Braitenverg cross-coupling
         v_l = 4/(1+br*exp(-b*oR(ts)));
         v_r = 4/(1+bl*exp(-b*oL(ts)));
        else
         v_l = oR(ts);
         v_r = oL(ts);
        end
    end
    vl(ts) = v_l;
    vr(ts) = v_r;
%     error(ts) = deg2rad(robot_pose(3)-spkr_angle);
    
    if (v_l ~= v_r)
        R = (l/2)*(v_r + v_l)/(v_r - v_l);
        robot_omega = (v_r - v_l)/l;
        icc = [robot_pose(1) - (R*sin(robot_pose(3))), ...
               robot_pose(2) + (R*cos(robot_pose(3)))];
        robot_pose = [cos(robot_omega) -sin(robot_omega) 0;...
                      sin(robot_omega)  cos(robot_omega) 0;...
                      0                 0                1]...
                      *...
                      [R*sin(robot_pose(3));...
                      -R*cos(robot_pose(3));...
                      robot_pose(3)]...
                      +...
                      [icc(1);...
                      icc(2);...
                      robot_omega];               
        
        if (abs(robot_pose(3)) > pi)
            robot_pose(3) = robot_pose(3) - sign(robot_pose(3))*2*pi;
        end
    else
        robot_omega = 0;
        robot_pose = robot_pose + ...
                    [cos(robot_pose(3)/v_l); ...
                     sin(robot_pose(3)/v_l); ...
                     0];
    end
    
    % Calculate distance travelled
    dist_moved(itr+1) = dist_moved(itr+1) + ...
                      sqrt((robot_pose(2)-last_robot_xy(2))^2 + ...
                           (robot_pose(2)-last_robot_xy(2))^2);
    
    
    a1 = plot(h1,robot_pose(1),robot_pose(2),'ok');
    if (learning_off == 1)
    if (mod(ts,5)== 0)
    a2(ts) = line([robot_pose(1),robot_pose(1)+5*cos(robot_pose(3))],...
         [robot_pose(2),robot_pose(2)+5*sin(robot_pose(3))],'Marker','.','LineWidth',5);
    end
    else
    a2(ts) = line([robot_pose(1),robot_pose(1)+5*cos(robot_pose(3))],...
         [robot_pose(2),robot_pose(2)+5*sin(robot_pose(3))],'LineWidth',4,'Color','green');
    end
    
    htxt1 = text(robot_pose(1)-150,robot_pose(2)-25,strcat('v_l = ',num2str(v_l)));
    htxt2 = text(robot_pose(1)-150,robot_pose(2)-45,strcat('v_r = ',num2str(v_r)));
    htxt3 = text(300,600-25,strcat('bl = ',num2str(bl)));
    htxt4 = text(300,600-45,strcat('br = ',num2str(br)));
%     htxt5 = text(0,-65,strcat('itr = ',num2str(itr)));
%     htxt6 = text(0,-85,strcat('ts = ',num2str(ts)));
    htxt5 = text(300,600-65,strcat('Iteration = ',num2str(itr)));
    htxt6 = text(300,600-85,strcat('Time step = ',num2str(ts)));
    htxt7 = text(300,600-105,strcat('Distance travelled = ',num2str(dist_moved(itr+1))));
    
    a3 = plot(h2,-2+dist_min/sensor(3),'g','LineWidth',2);
    a4 = plot(h2,oR,'b','LineWidth',2);
    a5 = plot(h2,oL,'r','LineWidth',2);
    a8 = plot(h4,vl,'m','LineWidth',2);
    a9 = plot(h4,vr,'k','LineWidth',2);
    
    if (record_avi == 1)
        frm = getframe(fh);
        writeVideo(writerObj,frm);
    end
    pause(0.0001);
    
    delete(htxt1);
    delete(htxt2);
    delete(htxt3);
    delete(htxt4);
    delete(htxt5);
    delete(htxt6);
    delete(htxt7);
    delete(a1);
    
    sensor = [robot_pose(1) robot_pose(2) sensor_threshold];
%     viscircles(sensor(1:2),sensor(3),'EdgeColor','k','LineStyle','-.');

    dist = sqrt((spkr_xy(2)-robot_pose(2))^2 + ...
                (spkr_xy(1)-robot_pose(1))^2);
            
    if (dist < 10)
        break;
    end
end

htxt3 = text(300,600-25,strcat('bl =',num2str(bl)));
htxt4 = text(300,600-45,strcat('br =',num2str(br)));
htxt5 = text(300,600-65,strcat('Iteration = ',num2str(itr)));
htxt6 = text(300,600-85,strcat('Time step = ',num2str(ts)));
htxt7 = text(300,600-105,strcat('Distance travelled = ',num2str(dist_moved(itr+1))));

if (record_avi == 1)
    close(writerObj);
end

% dist_moved
% delete(a2)

pause;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% bl = 26.8675; % Fig. 2
% br = 21.5936; %
% bl = 26.8465; % Fig. 7
% br = 8.5849; %
% bl = 41.8227; % Fig. 9
% br = 5.4423; %
% bl = 33.4815; % Fig. 10
% br = 14.8327; %
% bl = 54.5994; % Fig. 11
% br = 8.3348; %

if (record_avi == 1)
    writerObj = VideoWriter('3nav_no_obs_avoidance_learning.mp4','MPEG-4');
    open(writerObj);
%     loops = size(1:1000);
%     F(loops(1,2)) = struct('cdata',[],'colormap',[]);
end
delete(htxt3);
delete(htxt4);
delete(htxt5);
delete(htxt6);
delete(htxt7);
cla(h2);
cla(h4);
robot_pose = [0; 0; deg2rad(0)]; % [x y theta]
v_l = 0; % linear velocity of left wheel cm/s
v_r = 0; % linear velocity of right wheel cm/s
robot_omega = (v_r - v_l)/l;
sensor = [robot_pose(1) robot_pose(2) ...
          sensor_threshold sensor_threshold2]; % distance sensor
                                               % position and
                                               % thresholds
                                               % [x y t t2]
diff = [];
d_diff = [];
dist_min = [];
dist_moved(itr + 1) = 0;
oL = [];
oR = [];
vl = [];
vr = [];
% obst = [40+round((1.75*r-0.25*r)*rand(n_obst,2)) ...
%         round(5+((34000/freq)-5)*rand(n_obst,1))]; % obstacle location and radius
%                                          % [x y r]. Radius is between 5 and
%                                          % 34000/freq = sound wavelength.

plot(h1,spkr_xy(1),spkr_xy(2),'sk','MarkerFaceColor','black');
% plot(h1,obst(:,1),obst(:,2),'xr');
for i = 1:n_obst
    h(i) = viscircles(obst(i,1:2),obst(i,3));
    x = cell2mat(get(h(i).Children,'XData'));
    y = cell2mat(get(h(i).Children,'YData'));
    hf(i) = fill(x(2,1:end-1),y(2,1:end-1),'r');
end
theta = zeros(1);
plot(h1,robot_pose(1),robot_pose(2),'ok');
line([robot_pose(1),robot_pose(1)+5*cos(robot_pose(3))],...
[robot_pose(2),robot_pose(2)+5*sin(robot_pose(3))]);

learning_off = 0;
for ts = 1:1000 % in seconds
    
    last_robot_xy = [robot_pose(1) robot_pose(2)]; % last robot position
                                                   % used to calculate
                                                   % distance travelled
    
    % Determine speaker position relative to the robot
    spkr_angle = atan2(spkr_xy(2)-robot_pose(2),spkr_xy(1)-robot_pose(1));

    % Sense the sound direction with the ear model
    phase_diff = c*sin(robot_pose(3)-spkr_angle);

    if add_noise == 0
        sinewave_L = sin(omega*t - 0);
        sinewave_R = sin(omega*t - phase_diff);
    else
        sinewave_L = awgn(sin(omega*t - 0),snr_db);
        sinewave_R = awgn(sin(omega*t - phase_diff),snr_db);
    end

    Cfilter_output_L = filter(bz_C,az_C,sinewave_L);
    Ifilter_output_L = filter(bz_I,az_I,sinewave_L);
    Cfilter_output_R = filter(bz_C,az_C,sinewave_R);
    Ifilter_output_R = filter(bz_I,az_I,sinewave_R);

    outL = 20*log10(sum(abs(Ifilter_output_L(n_samples-n_op_samples+1:end) + ...
                            Cfilter_output_R(1,n_samples-n_op_samples+1:end))));
    outR = 20*log10(sum(abs(Ifilter_output_R(1,n_samples-n_op_samples+1:end) + ...
                            Cfilter_output_L(1,n_samples-n_op_samples+1:end))));

    if (intermittent_sound == 1)
        if (mod(ts,n) == 0)
            n = round(2+10*rand);
            oL(ts) = 0;
            oR(ts) = 0;
        else
            oL(ts) = 118-abs(outL);
            oR(ts) = 118-abs(outR);
        end
    else
        oL(ts) = 118-abs(outL);
        oR(ts) = 118-abs(outR);
    end
    
    % Determine the distance to the obstacles and find all obstacles within
    % distance sensor threshhold (20cm)
    for i = 1:n_obst
        dist(i,1) = sqrt((sensor(2)-obst(i,2))^2 + (sensor(1)-obst(i,1))^2) ...
                    - obst(i,3);
        if add_noise == 1
            dist(i,1) = awgn(dist(i,1),3);
        end
        if (dist(i,1) <= sensor(3))
            dist(i,2) = i;
        else
            dist(i,2) = 0;
        end
    end
    
    % Determine the distance to the closest obstacle
    if min(find(dist(:,2)))
        dist_min(ts) = dist(min(find(dist(:,2))),1);
    else
        dist_min(ts) = 0;
    end
    
    % If the obstacle is closer than the distance sensor threshold, perform
    % evasive manoeuver by determining whether the obstacle is the left or
    % right (not how much to left or right) and turning with fixed
    % velocity in the opposite direction.
    if min(find(dist(:,2)))
        theta = (atan2(obst(min(find(dist(:,2))),2)-sensor(2),...
                      obst(min(find(dist(:,2))),1)-sensor(1))-...
                      robot_pose(3));
        if ((theta < 0) && (theta > deg2rad(-90))) % right side of robot
            v_l = 0.1;
            v_r = 4;
        elseif ((theta > 0)  && (theta < deg2rad(90))) % left side of robot
            v_l = 4;
            v_r = 0.1;
        end
    else
        if (learning_off == 0)
        % no reflex, Braitenverg cross-coupling
         v_l = 4/(1+br*exp(-b*oR(ts)));
         v_r = 4/(1+bl*exp(-b*oL(ts)));
        else
         v_l = oR(ts);
         v_r = oL(ts);
        end
    end
    vl(ts) = v_l;
    vr(ts) = v_r;
%     error(ts) = deg2rad(robot_pose(3)-spkr_angle);
    
    if (v_l ~= v_r)
        R = (l/2)*(v_r + v_l)/(v_r - v_l);
        robot_omega = (v_r - v_l)/l;
        icc = [robot_pose(1) - (R*sin(robot_pose(3))), ...
               robot_pose(2) + (R*cos(robot_pose(3)))];
        robot_pose = [cos(robot_omega) -sin(robot_omega) 0;...
                      sin(robot_omega)  cos(robot_omega) 0;...
                      0                 0                1]...
                      *...
                      [R*sin(robot_pose(3));...
                      -R*cos(robot_pose(3));...
                      robot_pose(3)]...
                      +...
                      [icc(1);...
                      icc(2);...
                      robot_omega];               
        
        if (abs(robot_pose(3)) > pi)
            robot_pose(3) = robot_pose(3) - sign(robot_pose(3))*2*pi;
        end
    else
        robot_omega = 0;
        robot_pose = robot_pose + ...
                    [cos(robot_pose(3)/v_l); ...
                     sin(robot_pose(3)/v_l); ...
                     0];
    end
    
    % Calculate distance travelled
    dist_moved(itr+1) = dist_moved(itr+1) + ...
                      sqrt((robot_pose(2)-last_robot_xy(2))^2 + ...
                           (robot_pose(2)-last_robot_xy(2))^2);
    
    
    a1 = plot(h1,robot_pose(1),robot_pose(2),'ok');
    if (learning_off == 1)
    if (mod(ts,5)== 0)
    a2(ts) = line([robot_pose(1),robot_pose(1)+5*cos(robot_pose(3))],...
         [robot_pose(2),robot_pose(2)+5*sin(robot_pose(3))],'Marker','.','LineWidth',5);
    end
    else
    a2(ts) = line([robot_pose(1),robot_pose(1)+5*cos(robot_pose(3))],...
         [robot_pose(2),robot_pose(2)+5*sin(robot_pose(3))],'LineWidth',4,'Color','green');
    end
        
    htxt1 = text(robot_pose(1)-150,robot_pose(2)-25,strcat('v_l = ',num2str(v_l)));
    htxt2 = text(robot_pose(1)-150,robot_pose(2)-45,strcat('v_r = ',num2str(v_r)));
    htxt3 = text(300,600-25,strcat('bl = ',num2str(bl)));
    htxt4 = text(300,600-45,strcat('br = ',num2str(br)));
    htxt5 = text(300,600-65,strcat('Iteration = ',num2str(itr)));
    htxt6 = text(300,600-85,strcat('Time step = ',num2str(ts)));
    htxt7 = text(300,600-105,strcat('Distance travelled = ',num2str(dist_moved(itr+1))));
    
    a3 = plot(h2,-2+dist_min/sensor(3),'g','LineWidth',2);
    a4 = plot(h2,oR,'b','LineWidth',2);
    a5 = plot(h2,oL,'r','LineWidth',2);
    a8 = plot(h4,vl,'m','LineWidth',2);
    a9 = plot(h4,vr,'k','LineWidth',2);
    
    if (record_avi == 1)
        frm = getframe(fh);
        writeVideo(writerObj,frm);
%         F(ts) = getframe(fh);
    end
    pause(0.0001);
    
    delete(htxt1);
    delete(htxt2);
    delete(htxt3);
    delete(htxt4);
    delete(htxt5);
    delete(htxt6);
    delete(htxt7);
    delete(a1);
    
    sensor = [robot_pose(1) robot_pose(2) sensor_threshold];
%     viscircles(sensor(1:2),sensor(3),'EdgeColor','k','LineStyle','-.');

    dist = sqrt((spkr_xy(2)-robot_pose(2))^2 + ...
                (spkr_xy(1)-robot_pose(1))^2);
            
    if (dist < 10)
        break;
    end
end

htxt3 = text(300,600-25,strcat('bl = ',num2str(bl)));
htxt4 = text(300,600-45,strcat('br = ',num2str(br)));
htxt5 = text(300,600-65,strcat('Iteration = ',num2str(itr)));
htxt6 = text(300,600-85,strcat('Time step = ',num2str(ts)));
htxt7 = text(300,600-105,strcat('Distance travelled = ',num2str(dist_moved(itr+1))));

if (record_avi == 1)
    close(writerObj);
%     movie2avi(F,'nav3.avi');
end

% dist_moved
% delete(a2)

pause;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% bl = 26.8675; % Fig. 2
% br = 21.5936; %
% w = 2.8627;
% bl = 26.8465; % Fig. 7
% br = 8.5849; %
% w = 3.7752;
% bl = 41.8227; % Fig. 9
% br = 5.4423; %
% w = 2.2082;
% bl = 33.4815; % Fig. 10
% br = 14.8327; %
% w = 4.6555;
% bl = 54.5994; % Fig. 11
% br = 8.3348; %
% w = 3.6742;


while(1)
if (record_avi == 1)
    writerObj = VideoWriter('4nav_obs_avoidance_learned.mp4','MPEG-4');
    open(writerObj);
%     loops = size(1:1000);
%     F(loops(1,2)) = struct('cdata',[],'colormap',[]);
end
delete(htxt3);
delete(htxt4);
delete(htxt5);
delete(htxt6);
delete(htxt7);
cla(h2);
cla(h4);
robot_pose = [0; 0; deg2rad(0)]; % [x y theta]
v_l = 0; % linear velocity of left wheel cm/s
v_r = 0; % linear velocity of right wheel cm/s
robot_omega = (v_r - v_l)/l;
sensor = [robot_pose(1) robot_pose(2) ...
          sensor_threshold sensor_threshold2]; % distance sensor
                                               % position and
                                               % thresholds
                                               % [x y t t2]
diff = [];
d_diff = [];
dist_min = [];
dist_moved(itr + 1) = 0;
oL = [];
oR = [];
vl = [];
vr = [];

for i = 1:n_obst
    h(i) = viscircles(obst(i,1:2),obst(i,3));
    x = cell2mat(get(h(i).Children,'XData'));
    y = cell2mat(get(h(i).Children,'YData'));
    hf(i) = fill(x(2,1:end-1),y(2,1:end-1),'r');
end
theta = zeros(1);
plot(h1,robot_pose(1),robot_pose(2),'ok');
line([robot_pose(1),robot_pose(1)+5*cos(robot_pose(3))],...
[robot_pose(2),robot_pose(2)+5*sin(robot_pose(3))]);

off_time = 0;
off_t = 0;
rng('shuffle');
n = round(2+10*rand);

for ts = 1:1000 % in seconds
    
    last_robot_xy = [robot_pose(1) robot_pose(2)]; % last robot position
                                                   % used to calculate
                                                   % distance travelled
    
    % Determine speaker position relative to the robot
    spkr_angle = atan2(spkr_xy(2)-robot_pose(2),spkr_xy(1)-robot_pose(1));

    % Sense the sound direction with the ear model
    phase_diff = c*sin(robot_pose(3)-spkr_angle);

    if add_noise == 0
        sinewave_L = sin(omega*t - 0);
        sinewave_R = sin(omega*t - phase_diff);
    else
        sinewave_L = awgn(sin(omega*t - 0),snr_db);
        sinewave_R = awgn(sin(omega*t - phase_diff),snr_db);
    end

    Cfilter_output_L = filter(bz_C,az_C,sinewave_L);
    Ifilter_output_L = filter(bz_I,az_I,sinewave_L);
    Cfilter_output_R = filter(bz_C,az_C,sinewave_R);
    Ifilter_output_R = filter(bz_I,az_I,sinewave_R);

    outL = 20*log10(sum(abs(Ifilter_output_L(n_samples-n_op_samples+1:end) + ...
                            Cfilter_output_R(1,n_samples-n_op_samples+1:end))));
    outR = 20*log10(sum(abs(Ifilter_output_R(1,n_samples-n_op_samples+1:end) + ...
                            Cfilter_output_L(1,n_samples-n_op_samples+1:end))));

    if (intermittent_sound == 1)
        if (off_time > 0)
            off_time = off_time - 1;
%             oL(ts) = 0;
%             oR(ts) = 0;
            oL(ts) = oL(ts-1);
            oR(ts) = oR(ts-1);
        else
            if (mod(ts,n) == 0)
                n = round(30+(15-10)*rand);
                off_time = floor(1 + (10-0).*rand);
%                 oL(ts) = 0;
%                 oR(ts) = 0;
                oL(ts) = oL(ts-1);
                oR(ts) = oR(ts-1);
                if exist('htxt8')
                    delete(htxt8); 
                end
                htxt8 = text(300,600-125,'Sound off');
            else
                oL(ts) = 118-abs(outL);
                oR(ts) = 118-abs(outR);
                if exist('htxt8')
                    delete(htxt8); 
                end
                htxt8 = text(300,600-125,'Sound on');
                off_time = 0;
            end
        end
    else
        oL(ts) = 118-abs(outL);
        oR(ts) = 118-abs(outR);
        off_time = 0;
    end
%     if (intermittent_sound == 1)
%         if (mod(ts,n) == 0)
%             n = round(2+10*rand);
%             oL(ts) = 0;
%             oR(ts) = 0;
%         else
%             oL(ts) = 118-abs(outL);
%             oR(ts) = 118-abs(outR);
%         end
%     else
%         oL(ts) = 118-abs(outL);
%         oR(ts) = 118-abs(outR);
%     end
    
    % Determine the distance to the obstacles and find all obstacles within
    % distance sensor threshhold (20cm)
    for i = 1:n_obst
        dist(i,1) = sqrt((sensor(2)-obst(i,2))^2 + (sensor(1)-obst(i,1))^2) ...
                    - obst(i,3);
        if add_noise == 1
            dist(i,1) = awgn(dist(i,1),3);
        end
        if (dist(i,1) <= sensor(4))
            dist(i,3) = dist(i,1);
        else
            dist(i,3) = 0;
        end
        if (dist(i,1) <= sensor(3))
            dist(i,2) = dist(i,1);
        else
            dist(i,2) = 0;
        end
    end
    
    % Determine the distance to the closest obstacle
    if min(find(dist(:,2)))
        dist_min(ts) = dist(min(find(dist(:,2))),1);
    else
        dist_min(ts) = 0;
    end
    
    % If the obstacle is closer than the distance sensor threshold, perform
    % evasive manoeuver by determining whether the obstacle is the left or
    % right (not how much to left or right) and turning with fixed
    % velocity in the opposite direction.
    if min(find(dist(:,2)))
        theta = (atan2(obst(min(find(dist(:,2))),2)-sensor(2),...
                      obst(min(find(dist(:,2))),1)-sensor(1))-...
                      robot_pose(3));
        if ((theta < 0) && (theta > deg2rad(-90))) % right side of robot
            v_l = 0.1;
            v_r = 4;
        elseif ((theta > 0)  && (theta < deg2rad(90))) % left side of robot
            v_l = 4;
            v_r = 0.1;
        end
    else
        if (find(dist(:,3)))
            v = w*dist(min(find(dist(:,3))),3)/sensor(4) + dist_min(ts)/sensor(3);
            theta = (atan2(obst(min(find(dist(:,3))),2)-sensor(2),...
                      obst(min(find(dist(:,3))),1)-sensor(1))-...
                      robot_pose(3));
            if ((theta < 0) && (theta > deg2rad(-90))) % right side of robot
%                 v_l = 4/(1+b_test*exp(-b*oR(ts)));
%                 v_r = 4/(1+b_test*exp(-b*oL(ts))) + v;
                v_l = 4/(1+br*exp(-b*oR(ts)));
                v_r = 4/(1+bl*exp(-b*oL(ts))) + v;
            elseif ((theta > 0)  && (theta < deg2rad(90))) % left side of robot
%                 v_l = 4/(1+b_test*exp(-b*oR(ts))) + v;
%                 v_r = 4/(1+b_test*exp(-b*oL(ts)));
                v_l = 4/(1+br*exp(-b*oR(ts))) + v;
                v_r = 4/(1+bl*exp(-b*oL(ts)));
            else
%                 v_l = 4/(1+b_test*exp(-b*oR(ts)));
%                 v_r = 4/(1+b_test*exp(-b*oL(ts)));
                v_l = 4/(1+br*exp(-b*oR(ts)));
                v_r = 4/(1+bl*exp(-b*oL(ts)));
            end
        else
%                 v_l = 4/(1+b_test*exp(-b*oR(ts)));
%                 v_r = 4/(1+b_test*exp(-b*oL(ts)));
            v_l = 4/(1+br*exp(-b*oR(ts)));
            v_r = 4/(1+bl*exp(-b*oL(ts)));
        end
    end
    vl(ts) = v_l;
    vr(ts) = v_r;
%     error(ts) = deg2rad(robot_pose(3)-spkr_angle);
    
    if (v_l ~= v_r)
        R = (l/2)*(v_r + v_l)/(v_r - v_l);
        robot_omega = (v_r - v_l)/l;
        icc = [robot_pose(1) - (R*sin(robot_pose(3))), ...
               robot_pose(2) + (R*cos(robot_pose(3)))];
        robot_pose = [cos(robot_omega) -sin(robot_omega) 0;...
                      sin(robot_omega)  cos(robot_omega) 0;...
                      0                 0                1]...
                      *...
                      [R*sin(robot_pose(3));...
                      -R*cos(robot_pose(3));...
                      robot_pose(3)]...
                      +...
                      [icc(1);...
                      icc(2);...
                      robot_omega];               
        
        if (abs(robot_pose(3)) > pi)
            robot_pose(3) = robot_pose(3) - sign(robot_pose(3))*2*pi;
        end
    else
        robot_omega = 0;
        robot_pose = robot_pose + ...
                    [cos(robot_pose(3)/v_l); ...
                     sin(robot_pose(3)/v_l); ...
                     0];
    end
    
    % Calculate distance travelled
    dist_moved(itr+1) = dist_moved(itr+1) + ...
                      sqrt((robot_pose(2)-last_robot_xy(2))^2 + ...
                           (robot_pose(2)-last_robot_xy(2))^2);
    
    
    a1 = plot(h1,robot_pose(1),robot_pose(2),'ok');
    
    a2(ts) = line([robot_pose(1),robot_pose(1)+5*cos(robot_pose(3))],...
             [robot_pose(2),robot_pose(2)+5*sin(robot_pose(3))],'LineWidth',4,'Color','red');

    htxt1 = text(robot_pose(1)-150,robot_pose(2)-25,strcat('v_l = ',num2str(v_l)));
    htxt2 = text(robot_pose(1)-150,robot_pose(2)-45,strcat('v_r = ',num2str(v_r)));
    htxt3 = text(300,600-25,strcat('bl = ',num2str(bl)));
    htxt4 = text(300,600-45,strcat('br = ',num2str(br)));
    htxt5 = text(300,600-65,strcat('Iteration = ',num2str(itr)));
    htxt6 = text(300,600-85,strcat('Time step = ',num2str(ts)));
    htxt7 = text(300,600-105,strcat('Distance travelled = ',num2str(dist_moved(itr+1))));
    
    a3 = plot(h2,-2+dist_min/sensor(3),'g','LineWidth',2);
    a4 = plot(h2,oR,'b','LineWidth',2);
    a5 = plot(h2,oL,'r','LineWidth',2);
    a8 = plot(h4,vl,'m','LineWidth',2);
    a9 = plot(h4,vr,'k','LineWidth',2);
    
    if (record_avi == 1)
        frm = getframe(fh);
        writeVideo(writerObj,frm);
%         F(ts) = getframe(fh);
    end
    pause(0.0001);
    
    delete(htxt1);
    delete(htxt2);
    delete(htxt3);
    delete(htxt4);
    delete(htxt5);
    delete(htxt6);
    delete(htxt7);
    delete(a1);
    
    sensor = [robot_pose(1) robot_pose(2) sensor_threshold sensor_threshold2];
%     viscircles(sensor(1:2),sensor(3),'EdgeColor','k','LineStyle','-.');

    dist = sqrt((spkr_xy(2)-robot_pose(2))^2 + ...
                (spkr_xy(1)-robot_pose(1))^2);
            
    if (dist < 10)
        break;
    end
end

htxt3 = text(300,600-25,strcat('bl = ',num2str(bl)));
htxt4 = text(300,600-45,strcat('br = ',num2str(br)));
htxt5 = text(300,600-65,strcat('Iteration = ',num2str(itr)));
htxt6 = text(300,600-85,strcat('Time step = ',num2str(ts)));
htxt7 = text(300,600-105,strcat('Distance travelled = ',num2str(dist_moved(itr+1))));

if (record_avi == 1)
    close(writerObj);
%     movie2avi(F,'nav4.avi');
end
% dist_moved
pause;
delete(a2);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

while(1)
pause;
delete(htxt3);
delete(htxt4);
delete(htxt5);
delete(htxt6);
delete(htxt7);
cla(h1);
cla(h2);
cla(h4);
robot_pose = [0; 0; deg2rad(0)]; % [x y theta]
v_l = 0; % linear velocity of left wheel cm/s
v_r = 0; % linear velocity of right wheel cm/s
robot_omega = (v_r - v_l)/l;
sensor = [robot_pose(1) robot_pose(2) ...
          sensor_threshold sensor_threshold2]; % distance sensor
                                               % position and
                                               % thresholds
                                               % [x y t t2]
diff = [];
d_diff = [];
dist_min = [];
dist_moved(itr + 1) = 0;
oL = [];
oR = [];
vl = [];
vr = [];
obst = [round((1.75*r-0.7*r)*rand(n_obst,2)) ...
        round(5+((34000/freq)-5)*rand(n_obst,1))]; % obstacle location and radius
                                         % [x y r]. Radius is between 5 and
                                         % 34000/freq = sound wavelength.
for i = 1:n_obst
    if (sqrt((spkr_xy(1)-obst(i,1))^2 + (spkr_xy(2)-obst(i,2))^2) < sensor_threshold)
        obst(i,1:2) = [obst(i,1) obst(i,2)]+1.5*sensor_threshold;
    end
end

plot(h1,spkr_xy(1),spkr_xy(2),'sk','MarkerFaceColor','black');
plot(h1,obst(:,1),obst(:,2),'xr');
for i = 1:n_obst
    h(i) = viscircles(obst(i,1:2),obst(i,3));
    x = cell2mat(get(h(i).Children,'XData'));
    y = cell2mat(get(h(i).Children,'YData'));
    hf(i) = fill(x(2,1:end-1),y(2,1:end-1),'r');
end
theta = zeros(1);
plot(h1,robot_pose(1),robot_pose(2),'ok');
line([robot_pose(1),robot_pose(1)+5*cos(robot_pose(3))],...
[robot_pose(2),robot_pose(2)+5*sin(robot_pose(3))]);

for ts = 1:1000 % in seconds
    
    last_robot_xy = [robot_pose(1) robot_pose(2)]; % last robot position
                                                   % used to calculate
                                                   % distance travelled
    
    % Determine speaker position relative to the robot
    spkr_angle = atan2(spkr_xy(2)-robot_pose(2),spkr_xy(1)-robot_pose(1));

    % Sense the sound direction with the ear model
    phase_diff = c*sin(robot_pose(3)-spkr_angle);

    if add_noise == 0
        sinewave_L = sin(omega*t - 0);
        sinewave_R = sin(omega*t - phase_diff);
    else
        sinewave_L = awgn(sin(omega*t - 0),snr_db);
        sinewave_R = awgn(sin(omega*t - phase_diff),snr_db);
    end

    Cfilter_output_L = filter(bz_C,az_C,sinewave_L);
    Ifilter_output_L = filter(bz_I,az_I,sinewave_L);
    Cfilter_output_R = filter(bz_C,az_C,sinewave_R);
    Ifilter_output_R = filter(bz_I,az_I,sinewave_R);

    outL = 20*log10(sum(abs(Ifilter_output_L(n_samples-n_op_samples+1:end) + ...
                            Cfilter_output_R(1,n_samples-n_op_samples+1:end))));
    outR = 20*log10(sum(abs(Ifilter_output_R(1,n_samples-n_op_samples+1:end) + ...
                            Cfilter_output_L(1,n_samples-n_op_samples+1:end))));

    if (intermittent_sound == 1)
        if (mod(ts,n) == 0)
            n = round(2+10*rand);
            oL(ts) = 0;
            oR(ts) = 0;
        else
            oL(ts) = 118-abs(outL);
            oR(ts) = 118-abs(outR);
        end
    else
        oL(ts) = 118-abs(outL);
        oR(ts) = 118-abs(outR);
    end
    
    % Determine the distance to the obstacles and find all obstacles within
    % distance sensor threshhold (20cm)
    for i = 1:n_obst
        dist(i,1) = sqrt((sensor(2)-obst(i,2))^2 + (sensor(1)-obst(i,1))^2) ...
                    - obst(i,3);
        if add_noise == 1
            dist(i,1) = awgn(dist(i,1),3);
        end
        if (dist(i,1) <= sensor(4))
            dist(i,3) = dist(i,1);
        else
            dist(i,3) = 0;
        end
        if (dist(i,1) <= sensor(3))
            dist(i,2) = dist(i,1);
        else
            dist(i,2) = 0;
        end
    end
    
    % Determine the distance to the closest obstacle
    if min(find(dist(:,2)))
        dist_min(ts) = dist(min(find(dist(:,2))),1);
    else
        dist_min(ts) = 0;
    end
    
    % If the obstacle is closer than the distance sensor threshold, perform
    % evasive manoeuver by determining whether the obstacle is the left or
    % right (not how much to left or right) and turning with fixed
    % velocity in the opposite direction.
    if min(find(dist(:,2)))
        theta = (atan2(obst(min(find(dist(:,2))),2)-sensor(2),...
                      obst(min(find(dist(:,2))),1)-sensor(1))-...
                      robot_pose(3));
        if ((theta < 0) && (theta > deg2rad(-90))) % right side of robot
            v_l = 0.1;
            v_r = 4;
        elseif ((theta > 0)  && (theta < deg2rad(90))) % left side of robot
            v_l = 4;
            v_r = 0.1;
        end
    else
        if (find(dist(:,3)))
            v = w*dist(min(find(dist(:,3))),3)/sensor(4) + dist_min(ts)/sensor(3);
            theta = (atan2(obst(min(find(dist(:,3))),2)-sensor(2),...
                      obst(min(find(dist(:,3))),1)-sensor(1))-...
                      robot_pose(3));
            if ((theta < 0) && (theta > deg2rad(-90))) % right side of robot
                v_l = 4/(1+br*exp(-b*oR(ts)));
                v_r = 4/(1+bl*exp(-b*oL(ts))) + v;
            elseif ((theta > 0)  && (theta < deg2rad(90))) % left side of robot
                v_l = 4/(1+br*exp(-b*oR(ts))) + v;
                v_r = 4/(1+bl*exp(-b*oL(ts)));
            else
                v_l = 4/(1+br*exp(-b*oR(ts)));
                v_r = 4/(1+bl*exp(-b*oL(ts)));
            end
        else
            v_l = 4/(1+br*exp(-b*oR(ts)));
            v_r = 4/(1+bl*exp(-b*oL(ts)));
        end
    end
    vl(ts) = v_l;
    vr(ts) = v_r;
%     error(ts) = deg2rad(robot_pose(3)-spkr_angle);
    
    if (v_l ~= v_r)
        R = (l/2)*(v_r + v_l)/(v_r - v_l);
        robot_omega = (v_r - v_l)/l;
        icc = [robot_pose(1) - (R*sin(robot_pose(3))), ...
               robot_pose(2) + (R*cos(robot_pose(3)))];
        robot_pose = [cos(robot_omega) -sin(robot_omega) 0;...
                      sin(robot_omega)  cos(robot_omega) 0;...
                      0                 0                1]...
                      *...
                      [R*sin(robot_pose(3));...
                      -R*cos(robot_pose(3));...
                      robot_pose(3)]...
                      +...
                      [icc(1);...
                      icc(2);...
                      robot_omega];               
        
        if (abs(robot_pose(3)) > pi)
            robot_pose(3) = robot_pose(3) - sign(robot_pose(3))*2*pi;
        end
    else
        robot_omega = 0;
        robot_pose = robot_pose + ...
                    [cos(robot_pose(3)/v_l); ...
                     sin(robot_pose(3)/v_l); ...
                     0];
    end
    
    % Calculate distance travelled
    dist_moved(itr+1) = dist_moved(itr+1) + ...
                      sqrt((robot_pose(2)-last_robot_xy(2))^2 + ...
                           (robot_pose(2)-last_robot_xy(2))^2);
    
    
    a1 = plot(h1,robot_pose(1),robot_pose(2),'ok');
    
    a2 = line([robot_pose(1),robot_pose(1)+5*cos(robot_pose(3))],...
             [robot_pose(2),robot_pose(2)+5*sin(robot_pose(3))],'LineWidth',4,'Color','red');

    a3 = plot(h2,-2+dist_min/sensor(3),'g','LineWidth',2);
    a4 = plot(h2,oR,'b','LineWidth',2);
    a5 = plot(h2,oL,'r','LineWidth',2);
    a8 = plot(h4,vl,'m','LineWidth',2);
    a9 = plot(h4,vr,'k','LineWidth',2);
    
    htxt1 = text(robot_pose(1)-150,robot_pose(2)-25,strcat('v_l = ',num2str(v_l)));
    htxt2 = text(robot_pose(1)-150,robot_pose(2)-45,strcat('v_r = ',num2str(v_r)));
    htxt3 = text(300,600-25,strcat('bl = ',num2str(bl)));
    htxt4 = text(300,600-45,strcat('br = ',num2str(br)));
    htxt5 = text(300,600-65,strcat('itr = ',num2str(itr)));
    htxt6 = text(300,600-85,strcat('ts = ',num2str(ts)));
    pause(0.0001);
    delete(htxt1);
    delete(htxt2);
    delete(htxt3);
    delete(htxt4);
    delete(htxt5);
    delete(htxt6);
    delete(a1);
    
    sensor = [robot_pose(1) robot_pose(2) sensor_threshold sensor_threshold2];
%     viscircles(sensor(1:2),sensor(3),'EdgeColor','k','LineStyle','-.');

    dist = sqrt((spkr_xy(2)-robot_pose(2))^2 + ...
                (spkr_xy(1)-robot_pose(1))^2);
            
    if (dist < 10)
        break;
    end
end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% fig,itr,dBB,dNOA,dLOA,tBB,tNOA,tLOA
% 1,x,398.4612,521.9372,322.2452,163,258,130
% 2,7,360.606,355.7293,314.3685,111,163,124 best
% 3,x,321.3639,345.3075,329.422,104,158,126
% 4,x,375.4171,397.3633,325.3019,128,174,126
% 5,x,911.6652,1230.2128,427.5526,350,624,149
% 6,x,338.5945,333.5422,319.9515,104,135,109
% 7,13,330.9226,323.5315,315.878,100,133,113 best
% 8,x,339.6128,341.8049,317.4275,104,131,107
% 9,8,366.9885,330.448,329.3503,124,134,124 best
% 10,19,395.7359,357.7746,321.0109,108,155,124 best
% 11,12,472.8776,360.9392,325.8374,132,169,138 best

% Arena transformations
set(gco,'XLim',[-100 400],'YLim',[-100 400]);
set(gco,'XTick',[0 100 200 300 400],'YTick',[0 100 200 300 400]);
set(gco,'XTickLabel',[0 1 2 3 4],'YTickLabel',[0 1 2 3 4]);
set(gco,'FontName','Helvetica','FontSize',20);

% Braitenberg coupling transformations
set(gco,'XTick',[-10 -5 0 5 10],'YTick',[0 2 4]);
set(gco,'FontName','Helvetica','FontSize',20);
legend off;
box on;

% Wheel velocity transformations
set(gco,'XLim',[1 200]);
set(gco,'XTick',[1 50 100 150 200],'YTick',[0 4 8]);
set(gco,'FontName','Helvetica','FontSize',20);
box on;

% Input signals transformations
set(gco,'XLim',[1 200],'YLim',[-3 7]);
set(gco,'XTick',[1 50 100 150 200],'YTick',[-2 0 3 6]);
set(gco,'YTickLabel',[0 0 3 6]);
set(gco,'FontName','Helvetica','FontSize',20);
box on;

% new transformations
hndl = get(gco,'Children');
delete(hndl(3:end));
hndl = get(gco,'Children');
delete(hndl(1:3:end));delete(hndl(2:3:end));
hndl = get(gco,'Children');
delete(hndl(4:end));
set(gco,'Color',[0 200/255 0]);
set(gco,'YLim',[-4 8],'XLim',[1 200]);
set(gco,'YTick',[-3 -1 1 4.5 8])
set(gco,'YTickLabel',[0 1 0 4 8])
set(gco,'FontName','Helvetica','FontSize',20);
hndl = get(gco,'Children');
hndl(1).YData = hndl(1).YData + 1;
hndl(2).YData = hndl(2).YData + 1;
hndl(3).YData = hndl(3).YData + 2;
hndl(3).YData = hndl(3).YData * 2;
hndl(3).YData = hndl(3).YData - 3;
box on;
set(gco,'XLim',[-100 400],'YLim',[-100 400]);
set(gco,'XTick',[0 100 200 300 400],'YTick',[0 100 200 300 400]);
set(gco,'XTickLabel',[0 1 2 3 4],'YTickLabel',[0 1 2 3 4]);
set(gco,'FontName','Helvetica','FontSize',20);
hndl = get(gco,'Children');
for i = 1:numel(hndl)
    if (strcmp(hndl(i).Type,'line') == 1)
        if (hndl(i).Color == [0 0.4470 0.7410])
            delete(hndl(i));
        end
%         if (hndl(i).Color == [0 1 0])
%             delete(hndl(i));
%         end
    end
end

dist = [360.606,355.7293,314.3685;
    330.9226,323.5315,315.878;
    366.9885,330.448,329.3503;
    395.7359,357.7746,321.0109;
    472.8776,360.9392,325.8374;
    ];

plot(bar);
set(gco,'FaceColor',[0.3 0.5 1]);
set(gco,'FaceColor',[0.8 0.9 1]);
set(gco,'FontName','Helvetica','FontSize',20);
set(gco,'YTick',[0 100 200 300 400 500]);
set(gco,'YTickLabel',[0 1 2 3 4 5]);
set(gco,'LineWidth',2);
set(gco,'LineWidth',2);
set(gco,'LineWidth',2);
grid on;
legend('no learning','learning without path-smoothing',...
    'learned path-smoothing');
set(gco,'Box','off');
ylabel 'Motion path length (m)';
xlabel 'Trial';

dist_BB = [360.606,330.9226,366.9885,395.7359,472.8776;
365.9973,322.8139,360.4546,372.6400,504.0555;
372.1233,329.7385,393.5596,386.3340,469.6269;
374.0737,328.5422,378.1914,370.2323,462.0268;
375.1563,332.0722,375.1319,397.7476,495.9545;
349.2603,319.0221,357.6391,400.5751,512.3301;
368.7622,328.6333,412.9609,373.2590,451.7172;
395.2382,324.9801,362.3580,373.0026,478.1153;
347.2722,323.3864,392.8535,374.3824,475.4962;
397.3153,326.1618,387.3729,374.6674,507.6081;
];

dist_NOA = [355.7293,323.5315,330.448,357.7746,360.9392;
366.3700,324.4076,329.3283,359.5751,351.6254;
330.1846,312.8945,327.8910,354.0483,362.1734;
359.8416,320.1128,328.9143,356.6012,357.1503;
360.5849,314.3111,330.5952,357.4217,356.0603;
358.9323,321.5060,329.8347,376.8762,363.4344;
369.9454,317.5602,327.6440,374.7313,347.3252;
375.1626,320.8940,329.2137,376.5453,361.9104;
360.3641,313.2003,326.9673,376.0322,358.3272;
340.4149,326.6918,328.6783,355.0779,360.2009;
];

dist_LOA = [314.3685,315.878,329.3503,321.0109,325.8374;
313.6865,315.1234,327.3724,321.5177,329.6927;
313.7193,315.6480,327.3150,321.2896,330.3187;
312.3792,316.3201,326.7873,319.0982,327.6486;
312.9426,315.9924,326.3655,319.2038,327.8370;
313.5748,315.8053,325.4512,321.0551,329.9268;
312.2375,314.1608,327.9487,322.4757,330.4561;
313.2753,314.2628,324.6425,322.1701,324.9025;
314.1173,317.0369,327.4295,320.9746,326.7977;
314.3201,315.0166,324.3667,323.3311,330.0909;
];

avg = [mean(dist_BB)' mean(dist_NOA)' mean(dist_LOA)'];
sd = [std(dist_BB)' std(dist_NOA)' std(dist_LOA)'];

figure;
bar(avg);
hold on;
ngroups = size(avg,1);
nbars = size(avg,2);
groupwidth = min(0.8,nbars/(nbars + 1.5));
x = zeros(1,5);
for i = 1:nbars
    x = (1:ngroups) - (groupwidth/2) + (2*i-1) * (groupwidth / (2*nbars));
    errorbar(x,avg(:,i),sd(:,i),'sr');
end

set(gco,'FaceColor',[0.3 0.5 1]);
set(gco,'FaceColor',[0.8 0.9 1]);
set(gco,'FontName','Helvetica','FontSize',20);
set(gco,'YTick',[0 100 200 300 400 500]);
set(gco,'YTickLabel',[0 1 2 3 4 5]);
set(gco,'LineWidth',2);
set(gco,'LineWidth',2);
set(gco,'LineWidth',2);
grid on;
legend('no learning','learning without path-smoothing',...
    'learned path-smoothing');
set(gco,'Box','off');
ylabel 'Motion path length (m)';
xlabel 'Trial';
box on;


speed = [95,97,108,110,119,124; % Fig. 2
         87,94,96,100,108,113;  % Fig. 7
         94,100,105,108,114,125;% Fig. 9
         90,96,107,119,121,124; % Fig. 10
         97,107,117,120,134,138 % Fig. 11
        ];
    
iteration = [8,9,9,9,11,7;     % Fig. 2
             9,15,18,15,13,13; % Fig. 7
             10,11,7,9,7,8;    % Fig. 9
             21,21,22,22,24,19;% Fig. 10
             14,15,16,15,17,12 % Fig. 11
             ];
    
figure;
subplot(1,2,1);
plot(speed','LineWidth',3);
axis square; box on; grid on;
set(gca,'FontName','Helvetica','FontSize',20);

subplot(1,2,2);
plot(iteration','LineWidth',3);
axis square; box on; grid on;
set(gca,'FontName','Helvetica','FontSize',20);
