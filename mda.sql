CREATE TABLE `ict_main` (
  `step` int(3) NOT NULL auto_increment,
  `part_number` char(15) default NULL,
  `PCB_location` char(2) default NULL,
  `component` char(1) default NULL,
  `actual_value` char(10) default NULL,
  `std_value` char(10) default NULL,
  `measured_value` char(10) default NULL,
  `high_limit` char(10) default NULL,
  `low_limit` char(10) default NULL,
  `s_pin` char(5) default NULL,
  `m_pin` char(5) default NULL,
  PRIMARY KEY  (`step`)
) ENGINE=MyISAM AUTO_INCREMENT=2 DEFAULT CHARSET=latin1;

