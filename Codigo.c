#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/supervisor.h>

#define TIME_STEP 32
#define NUM_SENSORS 8
#define MAX_SPEED 6.28
#define NUM_BOXES 20
#define THRESHOLD_MOV 0.005
#define ESPERA_MOVIMENTO 60
#define MAX_COLISOES_ENROSCO 3
#define INTERVALO_ENROSCO_PASSOS 94
#define ENROSCO_DIST_LIMIAR 0.03
#define RANDOM_MOVEMENT_TIMER 100
#define UNSTUCK_BACKOFF_STEPS 20
#define UNSTUCK_TURN_STEPS 40
#define UNSTUCK_FORWARD_STEPS 30

int passos_desde_primeira_colisao = -1;
int colisoes_recentes = 0;
double ultima_vl = 0.0, ultima_vr = 0.0;
static double ultima_pos_robo[3] = {0};

void initialize_motors(WbDeviceTag *left_motor, WbDeviceTag *right_motor) {
    *left_motor = wb_robot_get_device("left wheel motor");
    *right_motor = wb_robot_get_device("right wheel motor");
    wb_motor_set_position(*left_motor, INFINITY);
    wb_motor_set_position(*right_motor, INFINITY);
    wb_motor_set_velocity(*left_motor, 0);
    wb_motor_set_velocity(*right_motor, 0);
}

void initialize_sensors(WbDeviceTag *prox) {
    char sensor_name[5];
    for (int i = 0; i < NUM_SENSORS; i++) {
        sprintf(sensor_name, "ps%d", i);
        prox[i] = wb_robot_get_device(sensor_name);
        wb_distance_sensor_enable(prox[i], TIME_STEP);
    }
}

void update_box_positions(WbNodeRef *caixas, double caixa_pos_ant[NUM_BOXES][3]) {
    char def[16];
    for (int i = 0; i < NUM_BOXES; i++) {
        sprintf(def, "CAIXA%d", i + 1);
        caixas[i] = wb_supervisor_node_get_from_def(def);
        const double *pos = (caixas[i]) ? wb_supervisor_node_get_position(caixas[i]) : NULL;
        if (pos) {
            caixa_pos_ant[i][0] = pos[0];
            caixa_pos_ant[i][1] = pos[1];
            caixa_pos_ant[i][2] = pos[2];
        }
    }
}

void unstuck_routine(WbDeviceTag left_motor, WbDeviceTag right_motor) {
    printf("Modo enrosco ativado! Executando rotina de desatolamento...\n");

    
    for (int i = 0; i < UNSTUCK_BACKOFF_STEPS; i++) {
        wb_motor_set_velocity(left_motor, -MAX_SPEED * 0.5);
        wb_motor_set_velocity(right_motor, -MAX_SPEED * 0.5);
        wb_robot_step(TIME_STEP);
    }

    
    int sentido = (rand() % 2 == 0) ? 1 : -1;
    double turn_speed = MAX_SPEED * (0.5 + ((double)(rand() % 50) / 100.0)); 
    for (int i = 0; i < UNSTUCK_TURN_STEPS; i++) {
        wb_motor_set_velocity(left_motor, sentido * turn_speed);
        wb_motor_set_velocity(right_motor, -sentido * turn_speed);
        wb_robot_step(TIME_STEP);
    }

    
    for (int i = 0; i < UNSTUCK_FORWARD_STEPS; i++) {
        wb_motor_set_velocity(left_motor, MAX_SPEED * 0.6);
        wb_motor_set_velocity(right_motor, MAX_SPEED * 0.6);
        wb_robot_step(TIME_STEP);
    }

    printf("Rotina de desatolamento concluída.\n");
}

void handle_collision(WbDeviceTag left_motor, WbDeviceTag right_motor, WbNodeRef *caixas, double caixa_pos_ant[NUM_BOXES][3]) {
    printf("COLIDIU!\n");
    for (int t = 0; t < ESPERA_MOVIMENTO; t++) wb_robot_step(TIME_STEP);

    for (int i = 0; i < NUM_BOXES; i++) {
        if (!caixas[i]) continue;
        const double *nova_pos = wb_supervisor_node_get_position(caixas[i]);
        if (nova_pos && (
            fabs(nova_pos[0] - caixa_pos_ant[i][0]) > THRESHOLD_MOV ||
            fabs(nova_pos[2] - caixa_pos_ant[i][2]) > THRESHOLD_MOV)) {

            printf("Caixa %d se moveu! DANÇA!\n", i + 1);
            while (true) {
                wb_motor_set_velocity(left_motor, MAX_SPEED);
                wb_motor_set_velocity(right_motor, -MAX_SPEED);
                wb_robot_step(TIME_STEP);
            }
        }
    }

    printf("Nenhuma caixa se moveu.\n");
    if (passos_desde_primeira_colisao < 0) {
        passos_desde_primeira_colisao = 0;
        colisoes_recentes = 1;
    } else {
        passos_desde_primeira_colisao++;
        colisoes_recentes++;
    }

    const double *pos_robo = wb_supervisor_node_get_position(wb_supervisor_node_get_self());
    double dx = pos_robo[0] - ultima_pos_robo[0];
    double dz = pos_robo[2] - ultima_pos_robo[2];
    double dist = sqrt(dx * dx + dz * dz);

    if (passos_desde_primeira_colisao == 1) {
        ultima_pos_robo[0] = pos_robo[0];
        ultima_pos_robo[1] = pos_robo[1];
        ultima_pos_robo[2] = pos_robo[2];
    }

    if (passos_desde_primeira_colisao > INTERVALO_ENROSCO_PASSOS) {
        passos_desde_primeira_colisao = -1;
        colisoes_recentes = 0;
    } else if (colisoes_recentes >= MAX_COLISOES_ENROSCO && dist < ENROSCO_DIST_LIMIAR) {
        unstuck_routine(left_motor, right_motor);
        passos_desde_primeira_colisao = -1;
        colisoes_recentes = 0;
    }
}

void random_movement(WbDeviceTag left_motor, WbDeviceTag right_motor, int *timer) {
    if (--(*timer) <= 0) {
        double v_left = ((double)(rand() % 100) / 100.0) * MAX_SPEED;
        double v_right = ((double)(rand() % 100) / 100.0) * MAX_SPEED;
        ultima_vl = v_left;
        ultima_vr = v_right;
        wb_motor_set_velocity(left_motor, v_left);
        wb_motor_set_velocity(right_motor, v_right);
        *timer = RANDOM_MOVEMENT_TIMER + rand() % 100;
    }
}

int main() {
    wb_robot_init();
    srand(time(NULL));

    WbDeviceTag left_motor, right_motor;
    initialize_motors(&left_motor, &right_motor);

    WbDeviceTag prox[NUM_SENSORS];
    initialize_sensors(prox);

    WbNodeRef caixas[NUM_BOXES];
    double caixa_pos_ant[NUM_BOXES][3];
    update_box_positions(caixas, caixa_pos_ant);

    int timer = RANDOM_MOVEMENT_TIMER;

    while (wb_robot_step(TIME_STEP) != -1) {
        printf("Timer: %d\n", timer);

        int bateu = 0;
        for (int i = 0; i < NUM_SENSORS; i++) {
            double val = wb_distance_sensor_get_value(prox[i]);
            if (val > 80.0) bateu = 1;
        }

        if (bateu) {
            handle_collision(left_motor, right_motor, caixas, caixa_pos_ant);
            timer = 0; 
        }

        random_movement(left_motor, right_motor, &timer);
    }

    wb_robot_cleanup();
    return 0;
}

