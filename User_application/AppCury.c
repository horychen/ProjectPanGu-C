#include <All_Definition.h>
#include <stdio.h>
#include <math.h>

REAL HIP_SHANK_FREQUENCY = 0.004;  // kHz = 1 ms
REAL TEST_HIP_KP = 0.6;
REAL TEST_SHANK_KP = 1.5;
REAL TEST_HIP_SPD_KP = 0.005;
REAL TEST_HIP_SPD_KI = 5e-6;

REAL HIP_MIN = 0.2269; //0.0981;
REAL HIP_MAX = 0.5585; //0.9075; // 娑撹桨绮堟稊鍫ｇ箹娑擃亝妲哥拹鐔烘畱閿涚喕锟藉奔绗朚AX濮ｆ摤IN鐏忓骏绱�

REAL SHANK_MIN = 0.4538;
REAL SHANK_MAX = 1.1170; //1.7990;

REAL CAN01_MIN = 48000;
REAL CAN01_MAX = 41000;

REAL CAN03_MIN = 14000;
REAL CAN03_MAX = 1000;

extern REAL TEST_HIP_POS_OUTLIMIT = 500;
extern REAL TEST_SHANK_POS_OUTLIMIT = 825;

extern REAL SHANK_POS_CONTROL_IQ = 0.5;
extern REAL HIP_POS_CONTROL_POS = 35000;

int CONTROLLER_TYPE = 1;

    REAL deg_four_bar_map_motor_encoder_angle;
REAL rad_four_bar_map_motor_encoder_angle = 0;
int32 cnt_four_bar_map_motor_encoder_angle = 0;

CURRENT_WEIGHT_TABLE current_weight_table = {
    .current = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0},
    .weight = {3.80, 4.60, 7.70, 11.10, 14.22, 15.48}
};

REAL get_current_from_weight(REAL weight)
{
    if (weight >= current_weight_table.weight[5])
    {
        return current_weight_table.current[5];
    }
    else if (weight < current_weight_table.weight[0])
    {
        return current_weight_table.current[0];
    }
    else
    {
        int i;
        for (i = 0; i < 5; i++)
        {
            if (weight >= current_weight_table.weight[i] && weight < current_weight_table.weight[i+1])
            {
                return linearInterpolate(weight,
                                         current_weight_table.weight[i],
                                         current_weight_table.weight[i+1],
                                         current_weight_table.current[i],
                                         current_weight_table.current[i+1]);
            }
        }
        return current_weight_table.current[0];
    }
}


#pragma DATA_SECTION(hip_shank_angle_table,"MYTABLE_3");
HIP_SHANK_ANGLE_TABLE hip_shank_angle_table = {
    .hip = {0.040295454579787664,0.03948415763562727,0.022866482453853275,0.006205019353828244,-0.02261456856680697,-0.0528293160508345,-0.08208572841345071,-0.10969297382891527,-0.13551526996115024,-0.1595748833929039,-0.18195729217082332,-0.20277138862990815,-0.22212985277180683,-0.24013962870197067,-0.25689780976312904,-0.27249040801749047,-0.2869926059398794,-0.3004697083929059,-0.31297836388592615,-0.32456782472322054,-0.33528113036067364,-0.34515616316901554,-0.3542265615353123,-0.3625224939329287,-0.3700713064397264,-0.3768980594771546,-0.38302596983950477,-0.38847677289347715,-0.3932710180248834,-0.39742830847387467,-0.40096749488131966,-0.40390683027706065,-0.40626409290776533,-0.40805668221772085,-0.4093016924296081,-0.4100159674863316,-0.41021614057126066,-0.40991866098859986,-0.40913981082900197,-0.4078957135448985,-0.4062023362980053,-0.404075487705507,-0.4015308123936658,-0.39858378356308255,-0.39524969457675696,-0.3915436503999282,-0.3874805595504956,-0.38307512706181734,-0.3783418488175079,-0.373295007491729,-0.3679486702191735,-0.3623166880268721,-0.3564126969848078,-0.350250120973307,-0.3438421759211346,-0.3372018753376144,-0.33034203694322445,-0.3232752901940968,-0.31601408449497936,-0.3085706979007032,-0.300957246116405,-0.2931856916204585,-0.2852678527498148,-0.277215412604497,-0.26903992764534435,-0.2607528358762381,-0.2523654645185027,-0.24388903710060975,-0.235334679900578,-0.22671342769139327,-0.21803622875141399,-0.2093139491120121,-0.20055737602378254,-0.19177722063054561,-0.18298411984722127,-0.17418863744355031,-0.16540126434070787,-0.15663241813217768,-0.14789244184395284,-0.13919160195234323,-0.13054008568032188,-0.12194799759577618,-0.11342535553701215,-0.10498208589269889,-0.09662801826507383,-0.08837287954666459,-0.08022628744217607,-0.07219774346849073,-0.06429662546692187,-0.04732652888335151,-0.0379992970939838,-0.03084029152341164,-0.024637060858658497,-0.01779622532953743,-0.011948208334833527,-0.005689217619607485,0.0002857661664057414,0.00614514921078128,0.011908487430616784,0.017564451605226787,0.023087017963770988,0.028446970568217184,0.03361773669147538,0.038577325229814066,0.04331027728699151,0.040295454579787664},
    .shank = {-0.885519616918762,-0.8855502902086003,-0.8400092758027017,-0.8080857612879758,-0.7512062062591479,-0.691208741354794,-0.634556155394053,-0.5824633671508231,-0.5344950263367557,-0.49019106721508954,-0.4491772474919891,-0.41114990438416116,-0.37585799191076735,-0.3430902017015322,-0.31266591849537545,-0.2844287381996179,-0.2582417140728574,-0.23398379417171059,-0.2115470979199136,-0.1908347958339861,-0.17175943115418757,-0.1542415711598147,-0.13820870869490653,-0.12359435665346262,-0.11033729348500598,-0.09838092849096669,-0.08767276328843758,-0.07816393130468254,-0.06980880118654118,-0.0625646330018786,-0.05639127837170885,-0.05125091740503297,-0.04710782665410579,-0.043928173364955934,-0.041679832136661016,-0.04033222077441519,-0.039856152663086286,-0.04022370342788158,-0.04140809000808883,-0.04338356056492572,-0.0461252938879186,-0.0496093071659109,-0.05381237115664313,-0.05871193192902986,-0.06428603847006205,-0.07051327554754057,-0.07737270130420022,-0.08484378913078335,-0.09290637342760384,-0.10154059891796959,-0.11072687322389037,-0.12044582245619502,-0.13067824960826277,-0.14140509557602707,-0.15260740265715464,-0.1642662804098711,-0.17636287377714696,-0.18887833340503965,-0.20179378810507417,-0.21509031942977705,-0.22874893834785512,-0.24275056402099723,-0.257076004697897,-0.2717059407528108,-0.286620909905608,-0.3018012946679233,-0.3172273120654154,-0.3328790056894048,-0.3487362401320306,-0.36477869785764233,-0.3809858785592663,-0.39733710104272574,-0.4138115076723052,-0.4303880714007848,-0.4470456053933793,-0.46376277523961223,-0.4805181137297529,-0.49729003815323053,-0.5140568700558783,-0.5307968573710478,-0.5474881988171953,-0.5641090704317506,-0.5806376540884266,-0.5970521678231651,-0.6133308977730707,-0.6294522315135567,-0.6453946925619427,-0.6611369758014052,-0.6766579835680331,-0.7170714339555071,-0.7283929334760704,-0.7387860435884301,-0.7498403182934796,-0.7631348534707103,-0.7746223872781368,-0.7868141818928838,-0.7985512019024041,-0.8101858777524751,-0.8216503052714643,-0.83287319706123,-0.8437938456740783,-0.8543637117335205,-0.8645440310528507,-0.8743034646592742,-0.8836275734954298,-0.885519616918762}
};

REAL linearInterpolate(REAL x, REAL x1, REAL x2, REAL y1, REAL y2)
{
    return y1 + (x - x1) * (y2 - y1) / (x2 - x1);
}

REAL hip_shank_angle_to_can(REAL angle, int type)
{
    if (type == HIP_TYPE)
    {
        return linearInterpolate(angle, HIP_MIN, HIP_MAX, CAN01_MIN, CAN01_MAX);
    }
    else
    {
        return linearInterpolate(angle, SHANK_MIN, SHANK_MAX, CAN03_MIN, CAN03_MAX);
    }
}

CURYCONTROLLER curycontroller = {
    .L1 = 0.4,
    .L2 = 0.39495,
    .theta1 = INIT_THETA1,
    .theta2 = INIT_THETA2,
    .dot_theta1 = 0.0,
    .dot_theta2 = 0.0,
    .T = 1.5,
    .height_limit = {0.66, 0.78},
    .C = {{0.0, 0.0}, {0.1, 1.0}, {0.21, 0.93}, {1.0, 1.0}},
    .order = BEZIER_ORDER};

REAL IECON_HEIGHT = 0.70;

typedef struct
{
    REAL x,y;
    /* data */
}Point;

Point A,B,C,D,O;
REAL a_length, b_length, c_length, offset_length, lead;
REAL joint_offset;

Point get_intersections(Point p0, REAL r0, Point p1, REAL r1){
    REAL d, a, h, x2, y2, x3, y3, x4, y4;
    d = sqrt((p1.x-p0.x)*(p1.x-p0.x) + (p1.y-p0.y)*(p1.y-p0.y));

    a = (r0*r0-r1*r1+d*d)/(2*d);
    h = sqrt(r0*r0-a*a);
    x2 = p0.x+a*(p1.x-p0.x)/d;
    y2 = p0.y+a*(p1.y-p0.y)/d;
    x3 = x2+h*(p1.y-p0.y)/d;
    y3 = y2-h*(p1.x-p0.x)/d;

    x4 = x2-h*(p1.y-p0.y)/d;
    y4 = y2+h*(p1.x-p0.x)/d;
    Point ans;
    // printf("%lf,%lf\n", x4, y4);
    if(y4 >= 0.0){
        ans.x = x4;
        ans.y = y4;
    }
    else{
        ans.x = x3;
        ans.y = y3;
    }
    return ans;
}

REAL PositionCal(REAL joint_position){
    REAL distance, circles, motor_position;
    joint_position = (-1.0 *joint_position+joint_offset);
    B.x = a_length*cos(joint_position);
    B.y = a_length*sin(joint_position);


    C = get_intersections(B, b_length, D, c_length);

    distance = sqrt((C.x - O.x)*(C.x - O.x) + (C.y - O.y)*(C.y - O.y)) - offset_length;
    // printf("%lf, %lf\n",O.x, O.y);
    circles = distance/lead;
    motor_position = (circles - (int)circles ) * 2.0 * M_PI;
    return motor_position;
}

REAL get_motorpos(REAL angle){
    REAL deg_four_bar_map_motor_encoder_angle = PositionCal(angle / 180.0 * M_PI) / M_PI * 180.0 + 198.592172;
    while (deg_four_bar_map_motor_encoder_angle > 360.0){
        deg_four_bar_map_motor_encoder_angle -= 360.0;
    }
    return deg_four_bar_map_motor_encoder_angle;
}

void calc_dot_theta_from_height(REAL height)
{
    REAL theta1_front, theta2_front;
    theta1_front = acos((pow(curycontroller.L1, 2) + pow(height, 2) - pow(curycontroller.L2, 2)) / (2 * curycontroller.L1 * height));
    theta2_front = acos((pow(curycontroller.L2, 2) + pow(height, 2) - pow(curycontroller.L1, 2)) / (2 * curycontroller.L2 * height));
    curycontroller.dot_theta1 = (theta1_front - curycontroller.theta1) / (curycontroller.T / 1000);
    curycontroller.dot_theta2 = (theta2_front - curycontroller.theta2) / (curycontroller.T / 1000);
}

void calc_theta_from_height(REAL height)
{
    curycontroller.theta1 = acos((pow(curycontroller.L1, 2) + pow(height, 2) - pow(curycontroller.L2, 2)) / (2 * curycontroller.L1 * height));
    curycontroller.theta2 = acos((pow(curycontroller.L2, 2) + pow(height, 2) - pow(curycontroller.L1, 2)) / (2 * curycontroller.L2 * height));
}

void reset_position()
{
    calc_theta_from_height(curycontroller.height_limit[1]);
}

void linear_controller(REAL t)
{
    REAL height, height_front, t_front;
    int period = (int)(t / curycontroller.T);
    t = t - period * curycontroller.T;
    if (period % 2 == 0)
    {
        height = curycontroller.height_limit[0] + (curycontroller.height_limit[1] - curycontroller.height_limit[0]) * t / curycontroller.T;
    }
    else
    {
        height = curycontroller.height_limit[1] - (curycontroller.height_limit[1] - curycontroller.height_limit[0]) * t / curycontroller.T;
    }
    t_front = t + (curycontroller.T / 1000);
    period += (int)(t_front / curycontroller.T);
    t_front -= ((int)(t_front / curycontroller.T)) * curycontroller.T;
    if (period % 2 == 0)
    {
        height_front = curycontroller.height_limit[0] + (curycontroller.height_limit[1] - curycontroller.height_limit[0]) * t_front / curycontroller.T;
    }
    else
    {
        height_front = curycontroller.height_limit[1] - (curycontroller.height_limit[1] - curycontroller.height_limit[0]) * t_front / curycontroller.T;
    }
    calc_theta_from_height(height);
    calc_dot_theta_from_height(height_front);
}

void sinusoidal_controller(REAL t)
{
    REAL height = (curycontroller.height_limit[1] - curycontroller.height_limit[0]) / 2 * sin(2 * PI * t / curycontroller.T) + (curycontroller.height_limit[1] + curycontroller.height_limit[0]) / 2;
    REAL height_front = (curycontroller.height_limit[1] - curycontroller.height_limit[0]) / 2 * sin(2 * PI * (t + (curycontroller.T / 1000)) / curycontroller.T) + (curycontroller.height_limit[1] + curycontroller.height_limit[0]) / 2;
    calc_theta_from_height(height);
    calc_dot_theta_from_height(height_front);
}

void get_bezier_points()
{
    REAL t;
    int i, j;
    for (i = 0; i <= BEZIER_TRACE_SIZE; i++)
    {
        t = (REAL)i / BEZIER_TRACE_SIZE;
        curycontroller.bezier_trace[i][0] = 0.0;
        curycontroller.bezier_trace[i][1] = 0.0;
        for (j = 0; j < curycontroller.order; j++)
        {
            curycontroller.bezier_trace[i][0] += curycontroller.C[j][0] * pow(1 - t, curycontroller.order - 1 - j) * pow(t, j);
            curycontroller.bezier_trace[i][1] += curycontroller.C[j][1] * pow(1 - t, curycontroller.order - 1 - j) * pow(t, j);
        }
    }
    for (i = 0; i < BEZIER_TRACE_SIZE; i++)
    {
        for (j = i + 1; j < BEZIER_TRACE_SIZE; j++)
        {
            if (curycontroller.bezier_trace[i][0] > curycontroller.bezier_trace[j][0])
            {
                REAL temp[2];
                temp[0] = curycontroller.bezier_trace[i][0];
                temp[1] = curycontroller.bezier_trace[i][1];
                curycontroller.bezier_trace[i][0] = curycontroller.bezier_trace[j][0];
                curycontroller.bezier_trace[i][1] = curycontroller.bezier_trace[j][1];
                curycontroller.bezier_trace[j][0] = temp[0];
                curycontroller.bezier_trace[j][1] = temp[1];
            }
        }
    }
}

REAL bezier_linear_interpoolation(REAL t)
{
    int i;
    for (i = 0; i < BEZIER_TRACE_SIZE; i++)
    {
        if (curycontroller.bezier_trace[i][0] <= t && t <= curycontroller.bezier_trace[i + 1][0])
        {
            REAL height = (curycontroller.bezier_trace[i][1] +
                           (curycontroller.bezier_trace[i + 1][1] - curycontroller.bezier_trace[i][1]) * (t - curycontroller.bezier_trace[i][0]) / (curycontroller.bezier_trace[i + 1][0] - curycontroller.bezier_trace[i][0])) *
                              (curycontroller.height_limit[1] - curycontroller.height_limit[0]) +
                          curycontroller.height_limit[0];
            return height;
        }
    }
    return curycontroller.height_limit[1];
}

void bezier_controller(REAL t)
{
    REAL height, height_front, t_front;
    int period = (int)(t / curycontroller.T);
    t = t - period * curycontroller.T;
    if (period % 2 == 0)
    {
        height = bezier_linear_interpoolation(t / curycontroller.T);
    }
    else
    {
        height = curycontroller.height_limit[1] - (curycontroller.height_limit[1] - curycontroller.height_limit[0]) * t / curycontroller.T;
    }
    t_front = t + (curycontroller.T / 1000);
    period = period + (int)(t_front / curycontroller.T);
    t_front -= (int)(t_front / curycontroller.T) * curycontroller.T;
    if (period % 2 == 0)
    {
        height_front = bezier_linear_interpoolation(t_front / curycontroller.T);
    }
    else
    {
        height_front = curycontroller.height_limit[1] - (curycontroller.height_limit[1] - curycontroller.height_limit[0]) * t_front / curycontroller.T;
    }
    calc_theta_from_height(height);
    calc_dot_theta_from_height(height_front);
}

void height_controller(REAL height)
{
    calc_theta_from_height(height);
    curycontroller.dot_theta1 = 0.0;
    curycontroller.dot_theta2 = 0.0;
}

void run_iecon_main(Uint64 t)
{
    REAL t_trans = (double)(t*0.0001);
    switch (CONTROLLER_TYPE)
    {
    case 0:
        linear_controller(t_trans);
        break;
    case 1:
        sinusoidal_controller(t_trans);
        break;
    case 2:
        bezier_controller(t_trans);
        break;
    case -1:
        reset_position();
        break;
    case 3:
        height_controller(IECON_HEIGHT);
    default:
        break;
    }
}
