use strict;

my $HEIGHT = 30;
my $WIDTH = 32;
my $hmeters = 7.1;             # physical height of trapezoid in meters
my $wmeters = 4.5;             # physical width of trapezoid in meters
my $vehicleoffset = 5.36;      # meters from vehicle to bottom of trapezoid

my @CURVATURE = (-1/20, -1/30, -1/60, 0, +1/60, +1/30, +1/20);
#printcsv();
print_shift_matrix();
print_road_curvature();
exit;

sub printcsv {
    for (my $y=0; $y<$HEIGHT; $y++) {
	print "$y";
	foreach my $curve (@CURVATURE) {
	    my $delta = compute_shift($y, $curve);
	    print ",$delta";
	}
	print "\n";
    }

}

sub print_shift_matrix {
    print "const int shift_matrix[NUM_CURVATURE_HYPOTHESES][SAMPLE_HEIGHT] =";
    print " {\n";
    my $notfirst = 0;
    foreach my $curve (@CURVATURE) {
	if ($notfirst) {
	    print ",\n";
	}
	print "    {";
	for (my $y=0; $y<$HEIGHT; $y++) {
	    my $delta = compute_shift($y, $curve);
	    if ($y) {
		print ", ";
	    }
	    print "$delta";
	}
	print "}";
	$notfirst = 1;
    } 
    print "\n};\n";
}
sub print_road_curvature {
    print "const double road_curvature[NUM_CURVATURE_HYPOTHESES] = {\n    ";
    my $notfirst = 0;
    foreach my $curve (@CURVATURE) {
	if ($notfirst) {
	    print ", ";
	}
	print "$curve";
	$notfirst = 1;
    }
    print "\n};\n";
}
sub compute_shift {
    my $y = shift;
    my $curve = shift;

	# compute shift
	my $acurve = abs($curve);
	my $delta = 0;
	if ($acurve > 0.0001) {

	    # radius in meters
	    my $radius = 1/$acurve;

	    # total y in meters
	    my $ym = $vehicleoffset + $y * $hmeters / $HEIGHT;

	    # x in meters
	    my $xm = sqrt($radius*$radius - $ym*$ym);

	    # delta in meters
	    $delta = $radius - $xm;

	    # delta in pixels
	    $delta = $delta * $WIDTH / $wmeters;

	    # truncate to integer
	    $delta = int($delta);

	    if ($curve < 0.0) {
		$delta = -1 * $delta;
	    }
	}

    return $delta;
}
