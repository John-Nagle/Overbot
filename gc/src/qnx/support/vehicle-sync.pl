#!/usr/bin/perl -w

#
# Khian Hao Lim, Team Overbot, Dec 2003
# This program reads a watchdog startfile and copies relavant files from
# the local development machine to each of the vehicle machines
#
# usage:
# > vehicle-sync.pl startfile -q
# -q for quiet operation (don't ask, just do)

use strict;

my $verbose = 0;
my $quiet = 0;
my $startfile = "";
my @lines;
my %node_file_hash;

sub help {
    print "Usage:\n";
    print "> vehicle-sync.pl <-q> startfile\n";

    exit 1;
}

sub handle_args {
    if (!$_[0]) {
	&help;
    }

    $startfile = $_[0];

    if ($_[1]) {
	if ($_[1] eq "-q") {
	    $quiet = 1;
	}
	else {
	    print "Unknown flag $_[1]\n";
	    &help;
	}
    }

    print "handle_args: startfile: $startfile\n";
    print "handle_args: quiet: $quiet\n";

}

sub handle_file {

    open(STARTFILE, $startfile) || die "Failed to open $startfile: $!\n";
    chomp (@lines = <STARTFILE>);

    #print @lines;
    foreach (@lines) {
	print "$_\n";
    }


}

print "Work in Progress; do not use this program yet\n";
exit(1);

&handle_args(@ARGV);
&handle_file;
