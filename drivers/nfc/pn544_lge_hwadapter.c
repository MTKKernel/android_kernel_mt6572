
#include "pn544_lge_hwadapter.h"

int pn544_get_hw_revision(void)
{
	return 0;
}

unsigned int pn544_get_irq_pin(struct pn544_dev *dev)
{
	return dev->client->irq;
}

int pn544_gpio_to_irq(struct pn544_dev *dev)
{
	return dev->client->irq;
}

void pn544_gpio_enable(struct pn544_dev *pn544_dev)
{
	return;
}

void pn544_shutdown_cb(struct pn544_dev *pn544_dev)
{
	return;
}

