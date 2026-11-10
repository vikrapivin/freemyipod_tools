/* Rename the nano 5G NAND calls inside ftl-nano5g.c. The nano 4G driver
 * already owns the names nand_read_page and nand_get_geometry. */
#define nand_read_page ftl_phys_read_page
#define nand_read_pages ftl_phys_read_pages
#define nand_block_row ftl_phys_block_row
#define nand_write_supported ftl_phys_write_supported
#define nand_get_geometry ftl_phys_get_geometry
