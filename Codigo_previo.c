#include <stdio.h>
#include <string.h>
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/supervisor.h>

#define TIME_STEP 64
#define QTDD_CAIXAS 25  // Altere conforme o total exato
#define DIST_MOV_MINIMA 0.01

int main() {
  wb_robot_init();

  // Recupera referência para o robô
  WbNodeRef robot_node = wb_supervisor_node_get_self();
  const double *initial_robot_pos = wb_supervisor_node_get_position(robot_node);

  // Motores
  WbDeviceTag motor_esq = wb_robot_get_device("left wheel motor");
  WbDeviceTag motor_dir = wb_robot_get_device("right wheel motor");

  wb_motor_set_position(motor_esq, INFINITY);
  wb_motor_set_position(motor_dir, INFINITY);
  wb_motor_set_velocity(motor_esq, 0);
  wb_motor_set_velocity(motor_dir, 0);

  // Referência para caixas
  WbNodeRef caixas[QTDD_CAIXAS];
  char nome[10];

  for (int i = 0; i < QTDD_CAIXAS; i++) {
    sprintf(nome, "CAIXA%02d", i);
    caixas[i] = wb_supervisor_node_get_from_def(nome);
    if (caixas[i] == NULL) printf("Falha ao localizar %s\n", nome);
  }

  int caixa_encontrada = -1;

  for (int i = 0; i < QTDD_CAIXAS && caixa_encontrada == -1; i++) {
    const double *pos_inicial = wb_supervisor_node_get_position(caixas[i]);

    // Aproxima e empurra a caixa
    wb_motor_set_velocity(motor_esq, 2.0);
    wb_motor_set_velocity(motor_dir, 2.0);

    for (int step = 0; step < 40; step++)  // tempo para empurrar
      wb_robot_step(TIME_STEP);

    wb_motor_set_velocity(motor_esq, 0);
    wb_motor_set_velocity(motor_dir, 0);

    // Aguarda a caixa estabilizar
    for (int step = 0; step < 10; step++)
      wb_robot_step(TIME_STEP);

    const double *pos_final = wb_supervisor_node_get_position(caixas[i]);

    double dx = pos_final[0] - pos_inicial[0];
    double dz = pos_final[2] - pos_inicial[2];
    double dist = dx * dx + dz * dz;

    printf("Caixa %02d movida? Dist^2 = %.4f\n", i, dist);

    if (dist > DIST_MOV_MINIMA * DIST_MOV_MINIMA) {
      caixa_encontrada = i;
      break;
    }

    // Robô gira para mudar de caixa
    wb_motor_set_velocity(motor_esq, -2.0);
    wb_motor_set_velocity(motor_dir, 2.0);
    for (int step = 0; step < 25; step++)
      wb_robot_step(TIME_STEP);
    wb_motor_set_velocity(motor_esq, 0);
    wb_motor_set_velocity(motor_dir, 0);
  }

  // Gira infinitamente se a caixa leve foi achada
  if (caixa_encontrada != -1) {
    printf("Caixa leve encontrada: CAIXA%02d\n", caixa_encontrada);
    while (wb_robot_step(TIME_STEP) != -1) {
      wb_motor_set_velocity(motor_esq, -3.0);
      wb_motor_set_velocity(motor_dir, 3.0);
    }
  } else {
    printf("Caixa leve NAO encontrada.\n");
  }

  wb_robot_cleanup();
  return 0;
}
