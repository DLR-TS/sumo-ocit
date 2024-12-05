# sumo-ocit
Tools for using OCIT data with Eclipse SUMO

call `ocit2SUMO.py --help` for options

The input *OCIT.xml* file, must be annoted before calling `ocit2SUMO`: Each `<Signalgruppe>` needs an entry of the form

```
            <Bemerkungen>
                <Bemerkung>0;1</Bemerkung>
            </Bemerkungen>
```

where the `<Bemerkung>` element encloses a `;`-separated list of one or more SUMO tls-indices. See [SUMO connection indices](https://sumo.dlr.de/docs/sumo-gui.html#connectivity).

# Notice on the use of the OCIT trademark and the OCIT Data Standard
- OCIT® is a registered trademark of the companies AVT STOYE, Siemens, Stührenberg and SWARCO.
- The OCIT-C Center to Center Data Standard is created and owned by the companies Siemens, AVT STOYE, SWARCO, Stührenberg, Schlothauer & Wauer GmbH & Co KG, PTV Planung Transport Verkehr AG, GEVAS software GmbH and Verkehrs-Systeme AG. To use this standard in your own software, a license from any of the above companies must be obtained.
- The tools provided in this repository are created using a non-exclusive non-transferable license from the licensing companies above.
