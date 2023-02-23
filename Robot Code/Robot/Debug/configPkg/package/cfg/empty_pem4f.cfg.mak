# invoke SourceDir generated makefile for empty.pem4f
empty.pem4f: .libraries,empty.pem4f
.libraries,empty.pem4f: package/cfg/empty_pem4f.xdl
	$(MAKE) -f C:\Users\david\workspace_vMilestonesFinalFinal\Team24_Milestone10_Rodriguez_Lopez_Stewart\Milestone6\Milestone6/src/makefile.libs

clean::
	$(MAKE) -f C:\Users\david\workspace_vMilestonesFinalFinal\Team24_Milestone10_Rodriguez_Lopez_Stewart\Milestone6\Milestone6/src/makefile.libs clean

