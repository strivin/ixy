#include <stdio.h>
#include <unistd.h>
#include <sys/time.h>

#include "driver/device.h"

const int BATCH_SIZE = 32;

static void hex_dump_8bit(unsigned char *buf, size_t len) 
{
	size_t i;

	printf("\npacket info:");
	for (i = 0; i < len; i++) {
		if ((i % 16) == 0) {
			printf("\n");
			printf("%p: ", buf + i);
		}
		printf(" %02x", *(buf + i) & 0xff);
	}
	printf("\n");
}

int main(int argc, char* argv[]) {
	int n_packets = -1;
	struct pkt_buf* bufs[BATCH_SIZE];

	if (argc < 2 || argc > 3) {
		printf("Usage: %s <pci bus id> [n packets]\n", argv[0]);
		return 1;
	}

	struct ixy_device* dev = ixy_init(argv[1], 1, 1, -1);
	if (argc == 3) {
		n_packets = atoi(argv[2]);
		printf("Capturing %d packets...\n", n_packets);
	} else {
		printf("Capturing packets...\n");
	}

	while (n_packets != 0) {
		uint32_t num_rx = ixy_rx_batch(dev, 0, bufs, BATCH_SIZE);

		for (uint32_t i = 0; i < num_rx && n_packets != 0; i++) {
			hex_dump_8bit(bufs[i]->data, bufs[i]->size);
			pkt_buf_free(bufs[i]);

			// n_packets == -1 indicates unbounded capture
			if (n_packets > 0) {
				n_packets--;
			}
		}
	}

	return 0;
}
