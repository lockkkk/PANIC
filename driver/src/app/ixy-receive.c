#define _GNU_SOURCE
#include <stdio.h>
#include <unistd.h>
#include <sys/time.h>
#include <pthread.h>
#include <stdlib.h>
#include <stdatomic.h>
#include <json-c/json.h>
#include "driver/device.h"
#include "driver/mqnic_type.h"

const int BATCH_SIZE = 64;
struct poll_struct {
    struct ixy_device* dev;
    uint32_t queue_id;
};

// From https://wiki.wireshark.org/Development/LibpcapFileFormat
typedef struct pcap_hdr_s {
	uint32_t magic_number;  /* magic number */
	uint16_t version_major; /* major version number */
	uint16_t version_minor; /* minor version number */
	int32_t  thiszone;      /* GMT to local correction */
	uint32_t sigfigs;       /* accuracy of timestamps */
	uint32_t snaplen;       /* max length of captured packets, in octets */
	uint32_t network;       /* data link type */
} pcap_hdr_t;

typedef struct pcaprec_hdr_s {
	uint32_t ts_sec;        /* timestamp seconds */
	uint32_t ts_usec;       /* timestamp microseconds */
	uint32_t incl_len;      /* number of octets of packet saved in file */
	uint32_t orig_len;      /* actual length of packet */
} pcaprec_hdr_t;

typedef struct {
    _Atomic long total_count;
    _Atomic long total_byte_count;
    struct timeval last_print_time;
} SharedStats;

SharedStats stats[10];

void *poll_queue(void *context) {
	struct poll_struct * poll_info = (struct poll_struct *)context;
	struct ixy_device* dev = poll_info->dev;
	uint32_t queue_id = poll_info->queue_id;

//printf("Start Tenant %d Polling Thread \n", queue_id);
	// register_app(dev, queue_id, 0);

	struct timeval startt, endt;
	
	long total_count = 0;
	// these two number are used to measure the avg batch size
	long total_batch_num = 0, total_rx_batch_num = 0;
	long total_returned_count = 0, total_rx_returned_count = 0;

	long total_byte_count= 0;
	struct pkt_buf* bufs[BATCH_SIZE];
	uint64_t cur_id = 0, last_id = 0;

	gettimeofday(&startt, NULL);
	int batched_feedback = 0;

	while (true) {
		uint32_t num_rx = mqnic_rx_batch(dev, queue_id, bufs, BATCH_SIZE);
		struct timeval tv;
		batched_feedback += num_rx;
		
		if(num_rx!= 0){
			// printf("Receive Batch: %d\n", num_rx);
			total_rx_batch_num += num_rx;
			total_rx_returned_count +=  1;
		}

		for (uint32_t i = 0; i < num_rx; i++) {
			total_count ++;
			total_byte_count += bufs[i]->size;
			pkt_buf_free(bufs[i]);

		}

		total_batch_num += num_rx;
		total_returned_count += 1;
		// mqnic_rx_feedback(dev, queue_id, 1, num_rx);
		batched_feedback -= num_rx;

		// if(total_byte_count > 1000000000){
		// 	gettimeofday(&endt, NULL);
		// 	long msecs_time = ((endt.tv_sec - startt.tv_sec) * 1000000.0) + ((endt.tv_usec - startt.tv_usec));
		// 	printf( "Queue: %d, MBytes: %f, throughput: %f Gbps, PPS: %f Mpps, Avg Feedback Batch %f, Avg RX Batch %f\n", queue_id, total_byte_count  * 8.0/ 1000000 ,  total_byte_count * 8.0/ (msecs_time * 1000), total_count * 1.0/ msecs_time, total_batch_num  * 1.0 / total_returned_count,  total_rx_batch_num  * 1.0 / total_rx_returned_count);
		// 	total_batch_num = 0;
		// 	total_returned_count = 0;
		// 	total_count = 0;
		// 	total_byte_count = 0;
		// 	gettimeofday(&startt, NULL);
		// }

		if(total_byte_count > 1000000){
			// printf("Add %d\n", queue_id);
			atomic_fetch_add(&stats[queue_id].total_count, total_count);
            atomic_fetch_add(&stats[queue_id].total_byte_count, total_byte_count);
			total_batch_num = 0;
			total_returned_count = 0;
			total_count = 0;
			total_byte_count = 0;
		}


	}
}

typedef struct {
    int id;
    int prio;
    int *offloadChain;     // Now an array of integers
    int offloadChainSize;
    char *coreMask;
    char *ip;
    int port;
} TenantConfig;


typedef struct {
    char *pcieAddr;
    int coreCount;
    char *schePolicy;
    int engineNum;
    TenantConfig *tenants;
    int numTenants;
} Config;

Config parseConfig(const char *filename) {
    FILE *fp = fopen(filename, "r");
    if (!fp) {
        perror("Unable to open file");
        exit(1);
    }

    char buffer[2048];
    fread(buffer, sizeof(buffer), 1, fp);
    fclose(fp);

    struct json_object *parsed_json;
    struct json_object *driverconfig, *panicconfig, *tenantconfig;
    struct json_object *pcieaddr, *corecount, *sche_policy, *engine_num;
    struct json_object *tenant, *id, *prio, *offloadchain, *coremask, *ip, *port;

    parsed_json = json_tokener_parse(buffer);

    Config config = {0};
    config.tenants = NULL;
    config.numTenants = 0;

    // Parse driverconfig
    json_object_object_get_ex(parsed_json, "driverconfig", &driverconfig);
    json_object_object_get_ex(driverconfig, "pcieaddr", &pcieaddr);
    json_object_object_get_ex(driverconfig, "corecount", &corecount);
    config.pcieAddr = strdup(json_object_get_string(pcieaddr));
    config.coreCount = json_object_get_int(corecount);

    // Parse panicconfig
    json_object_object_get_ex(parsed_json, "panicconfig", &panicconfig);
    json_object_object_get_ex(panicconfig, "sche_policy", &sche_policy);
    json_object_object_get_ex(panicconfig, "engine_num", &engine_num);
    config.schePolicy = strdup(json_object_get_string(sche_policy));
    config.engineNum = json_object_get_int(engine_num);

    // Parse tenantconfig
    json_object_object_get_ex(parsed_json, "tenantconfig", &tenantconfig);
    json_object_object_foreach(tenantconfig, key, val) {
        config.tenants = realloc(config.tenants, sizeof(TenantConfig) * (config.numTenants + 1));
        tenant = val;
        json_object_object_get_ex(tenant, "id", &id);
        json_object_object_get_ex(tenant, "prio", &prio);
        json_object_object_get_ex(tenant, "offloadchain", &offloadchain);
        json_object_object_get_ex(tenant, "coremask", &coremask);
        json_object_object_get_ex(tenant, "ip", &ip);
        json_object_object_get_ex(tenant, "port", &port);

        TenantConfig *currentTenant = &config.tenants[config.numTenants];
        currentTenant->id = json_object_get_int(id);
        currentTenant->prio = json_object_get_int(prio);
        currentTenant->coreMask = strdup(json_object_get_string(coremask));
        currentTenant->ip = strdup(json_object_get_string(ip));
        currentTenant->port = json_object_get_int(port);

        int chain_length = json_object_array_length(offloadchain);
        currentTenant->offloadChain = malloc(chain_length * sizeof(int));
        currentTenant->offloadChainSize = chain_length;
        for (int j = 0; j < chain_length; j++) {
            struct json_object *chain_elem = json_object_array_get_idx(offloadchain, j);
            currentTenant->offloadChain[j] = json_object_get_int(chain_elem);
        }

        config.numTenants++;
    }

    json_object_put(parsed_json);
    return config;
}

void *printstats(void *arg) {
    struct timeval now;
    gettimeofday(&stats[0].last_print_time, NULL);

    while (true) {
        sleep(3);  // Wait for 2 seconds
        gettimeofday(&now, NULL);
        long seconds_elapsed = now.tv_sec - stats[0].last_print_time.tv_sec;
		long usecs_elapsed = ((now.tv_sec - stats[0].last_print_time.tv_sec) * 1000000L) + (now.tv_usec - stats[0].last_print_time.tv_usec);
        if (seconds_elapsed >= 2) {
            // Snapshot current stats
			for(int i =0; i < 3; i++){
				long total_count = atomic_exchange(&stats[i].total_count, 0);
				long total_byte_count = atomic_exchange(&stats[i].total_byte_count, 0);
				double mbits = (total_byte_count * 8.0) / 1000000.0;
				double throughput = mbits / (usecs_elapsed / 1000000.0);
				double pps = total_count / (usecs_elapsed / 1000000.0);
				printf("Tenant %d, Throughput: %.2f Gbps\n", i+1, throughput/1000.0);
			}
			printf("------------------------\n");
            stats[0].last_print_time = now;
        }
    }
}

int main(int argc, char* argv[]) {
	cpu_set_t cpuset;
	pthread_t thread[MQNIC_USER_QUEUE_NUMBER];
	struct poll_struct poll_info[MQNIC_USER_QUEUE_NUMBER];
	pthread_t stats_thread;

	
	Config config = parseConfig("config/config.json");
	struct ixy_device* dev = ixy_init(config.pcieAddr, config.coreCount, 0, 0);
	printf("Finish Start user space driver...\n");

	printf("Loading Config into PANIC...\n");
	sleep(4);
	for(int i = 0; i < config.numTenants; i++){
		int s = config.tenants[i].offloadChainSize;
		int chain = 0;
		if(s > 0){
			chain = config.tenants[i].offloadChain[s-1];
		}
		config_panic_mat(dev, config.tenants[i].id, chain, 26, config.tenants[i].prio, config.tenants[i].port);
	}

	for(int i = 0; i < config.coreCount; i++){
		CPU_ZERO(&cpuset);
        CPU_SET(i, &cpuset);

		poll_info[i].dev = dev;
		poll_info[i].queue_id = i;
		if (pthread_create(&thread[i], NULL, poll_queue, &poll_info[i] ))
        {
            printf("Create Thread Error\n");
        }

		if(pthread_setaffinity_np(thread[i], sizeof(cpu_set_t), &cpuset))
        {
            printf("Set affinity Error\n");
        }
	}

	pthread_create(&stats_thread, NULL, printstats, NULL);

	for(int i = 0; i < 1; i++){
		pthread_join(thread[i], NULL);
	}
	return 0;
}
