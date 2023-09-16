/***/
#ifndef __LOGGER__H
#define __LOGGER__H

#ifdef __cplusplus
extern "C" {
#endif

/* Comments and brief descriptions in logger.c file.*/
void log_temp_master(char *timestring, uint8_t address, double temperature, int32_t utemperature);
void log_press_master(char *timestring, uint8_t address, uint64_t pressure, uint64_t upressure);
void log_temp_slave(char *timestring, uint8_t address, double temperature);
void log_press_slave(char *timestring, uint8_t address, uint64_t pressure);
void log_tmeaserr_master(char *timestring, uint8_t address);
void log_pmeaserr_master(char *timestring, uint8_t address);
void log_measerr_master(char *timestring, uint8_t address);
void log_initerr_master(char *timestring, uint8_t address);
void log_temp_count();
void log_press_count();
void log_ierr_count();
void log_terr_count();
void log_perr_count();
void log_err_count();
void set_log_curve(size_t size, double *temperatures, uint64_t *pressures);
void delete_log_curve(void);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __LOGGER__H  */
