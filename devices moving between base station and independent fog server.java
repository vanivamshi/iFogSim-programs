package org.fog.test.perfeval;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

import org.apache.commons.math4.legacy.core.Pair;
import org.cloudbus.cloudsim.Host;
import org.cloudbus.cloudsim.Log;
import org.cloudbus.cloudsim.Pe;
import org.cloudbus.cloudsim.Storage;
import org.cloudbus.cloudsim.core.CloudSim;
import org.cloudbus.cloudsim.power.PowerHost;
import org.cloudbus.cloudsim.provisioners.RamProvisionerSimple;
import org.cloudbus.cloudsim.sdn.overbooking.BwProvisionerOverbooking;
import org.cloudbus.cloudsim.sdn.overbooking.PeProvisionerOverbooking;
import org.fog.application.AppEdge;
import org.fog.application.AppLoop;
import org.fog.application.Application;
import org.fog.application.selectivity.FractionalSelectivity;
import org.fog.entities.Actuator;
import org.fog.entities.FogBroker;
import org.fog.entities.FogDevice;
import org.fog.entities.FogDeviceCharacteristics;
import org.fog.entities.MicroserviceFogDevice;
import org.fog.entities.PlacementRequest;
import org.fog.entities.Sensor;
import org.fog.entities.Tuple;
import org.fog.mobilitydata.DataParser;
import org.fog.mobilitydata.RandomMobilityGenerator;
import org.fog.mobilitydata.References;
import org.fog.placement.Controller;
import org.fog.placement.LocationHandler;
import org.fog.placement.MicroservicesMobilityClusteringController;
import org.fog.placement.ModuleMapping;
import org.fog.placement.ModulePlacementEdgewards;
import org.fog.placement.ModulePlacementMapping;
import org.fog.placement.PlacementLogicFactory;
import org.fog.policy.AppModuleAllocationPolicy;
import org.fog.scheduler.StreamOperatorScheduler;
import org.fog.utils.FogLinearPowerModel;
import org.fog.utils.FogUtils;
import org.fog.utils.TimeKeeper;
import org.fog.utils.distribution.DeterministicDistribution;
import org.json.simple.parser.ParseException;
public class TwoApps {
	static List<FogDevice> fogDevices1 = new ArrayList<FogDevice>();
	static List<FogDevice> fogDevices2 = new ArrayList<FogDevice>();
	static List<Sensor> sensors = new ArrayList<Sensor>();
	static List<Actuator> actuators = new ArrayList<Actuator>();
	static int numOfAreas = 2;
	static int numOfCamerasPerArea1=5;
	static int numberOfMobileUser = 1; // mobile users moving into independent fog server
    static double SENSOR_TRANSMISSION_TIME = 10; //transmission time of mobile users moving into independent fog server
	static double CAM_TRANSMISSION_TIME = 5;
	private static boolean CLOUD = false;
	
    static Map<Integer, Integer> userMobilityPattern = new HashMap<Integer, Integer>();
    static LocationHandler locator;

    // if random mobility generator for users is True, new random dataset will be created for each user
    static boolean randomMobility_generator = false; // To use random datasets
    static boolean renewDataset = false; // To overwrite existing random datasets
    static List<Integer> clusteringLevels = new ArrayList<Integer>(); // The selected fog layers for clustering

    //application
    static List<Application> applications = new ArrayList<>();
    static List<Pair<Double, Double>> qosValues = new ArrayList<>();

    
	public static void main(String[] args) {
		Log.printLine("Starting smart car parking system...");
		try {
			Log.disable();
			int num_user = 1; // number of cloud users
			Calendar calendar = Calendar.getInstance();
			boolean trace_flag = false; // mean trace events
			CloudSim.init(num_user, calendar, trace_flag);
			
			String appId1 = "dcns1"; // identifier of the application
			
			FogBroker broker1 = new FogBroker("broker1");
			
            /**
             * Microservices-based application creation - a single application is selected for this
             */
            Application microservicesApplication = createApplication1("example", broker1.getId());
            applications.add(microservicesApplication);

            //
            DataParser dataObject = new DataParser();
            locator = new LocationHandler(dataObject);

            String datasetReference = References.dataset_reference;

            if (randomMobility_generator) {
                datasetReference = References.dataset_random;
                createRandomMobilityDatasets(References.random_walk_mobility_model, datasetReference, renewDataset);
            }


            /**
             * Clustered Fog node creation.
             * 01. Create devices (Client,FON,FCN,Cloud)
             * 02. Generate cluster connection.
             * 03. Identify devices monitored by each FON
             */
            createMobileUser(broker1.getId(), applications.get(0).getAppId(), datasetReference); //sensors in our case
            createFogDevices(broker1.getId(), applications.get(0).getAppId(), datasetReference);

            /**
             * Central controller for performing preprocessing functions
             */
            List<Application> appList = new ArrayList<>();
            for (Application application : applications)
                appList.add(application);


            List<Integer> clusterLevelIdentifier = new ArrayList<>();
            clusterLevelIdentifier.add(2);

            int placementAlgo = PlacementLogicFactory.CLUSTERED_MICROSERVICES_PLACEMENT;
            MicroservicesMobilityClusteringController microservicesController = new MicroservicesMobilityClusteringController("controller", fogDevices1, sensors, appList, clusterLevelIdentifier, 2.0, placementAlgo, locator);

            // generate placement requests
            List<PlacementRequest> placementRequests = new ArrayList<>();
            for (Sensor s : sensors) {
                Map<String, Integer> placedMicroservicesMap = new HashMap<>();
                placedMicroservicesMap.put("clientModule", s.getGatewayDeviceId());
                PlacementRequest p = new PlacementRequest(s.getAppId(), s.getId(), s.getGatewayDeviceId(), placedMicroservicesMap);
                placementRequests.add(p);
            }

            microservicesController.submitPlacementRequests(placementRequests, 0);

            
			//Application application1 = createApplication1(appId1, broker1.getId());

            microservicesApplication.setUserId(broker1.getId());
			
			//createFogDevices(broker1.getId(), appId1, datasetReference);
			
			Controller controller = null;
			ModuleMapping moduleMapping1 = ModuleMapping.createModuleMapping(); // initializing a module mapping
			
			for(FogDevice device : fogDevices1){
				if(device.getName().startsWith("c")){ // names of all Smart Cameras start with 'm' 
					moduleMapping1.addModuleToDevice("picture-capture", device.getName());  // fixing 1 instance of the Motion Detector module to each Smart Camera
				}
			}

			for(FogDevice device : fogDevices1){
				if(device.getName().startsWith("a")){ // names of all fog devices start with 'a' 
					moduleMapping1.addModuleToDevice("slot-detector", device.getName());  // fixing 1 instance of the Motion Detector module to each Smart Camera
				}
			}

			//moduleMapping.addModuleToDevice("user_interface", "cloud"); // fixing instances of User Interface module in the Cloud
			if(CLOUD){
				// if the mode of deployment is cloud-based
				moduleMapping1.addModuleToDevice("picture-capture", "cloud"); // placing all instances of Object Detector module in the Cloud
				moduleMapping1.addModuleToDevice("slot-detector", "cloud"); // placing all instances of Object Tracker module in the Cloud
			}
			
			//controller = new Controller("master-controller", fogDevices1, sensors, 
			//		actuators);

			
			//controller.submitApplication(microservicesApplication, 
			//		(CLOUD)?(new ModulePlacementMapping(fogDevices1, microservicesApplication, moduleMapping1))
			//				:(new ModulePlacementEdgewards(fogDevices1, sensors, actuators, microservicesApplication, moduleMapping1)));


			TimeKeeper.getInstance().setSimulationStartTime(Calendar.getInstance().getTimeInMillis());
			
			CloudSim.startSimulation();

			CloudSim.stopSimulation();

			Log.printLine("VRGame finished!");
		} catch (Exception e) {
			e.printStackTrace();
			Log.printLine("Unwanted errors happen");
		}
	}
	
	
	
    private static void createRandomMobilityDatasets(int mobilityModel, String datasetReference, boolean renewDataset) throws IOException, ParseException {
        RandomMobilityGenerator randMobilityGenerator = new RandomMobilityGenerator();
        for (int i = 0; i < numOfCamerasPerArea1; i++) {

            randMobilityGenerator.createRandomData(mobilityModel, i + 1, datasetReference, renewDataset);
        }
    }

    
	/**
	 * Creates the fog devices in the physical topology of the simulation.
	 * @param userId
	 * @param appId
	 */
    private static void createMobileUser(int userId, String appId, String datasetReference) throws IOException {

        for (int id = 1; id <= numberOfMobileUser; id++)
            userMobilityPattern.put(id, References.DIRECTIONAL_MOBILITY);

        locator.parseUserInfo(userMobilityPattern, datasetReference);

        List<String> mobileUserDataIds = locator.getMobileUserDataId();

        for (int i = 0; i < numberOfMobileUser; i++) {
            FogDevice mobile = addMobile("mobile_" + i, userId, appId, References.NOT_SET); // adding mobiles to the physical topology. Smartphones have been modeled as fog devices as well.
            mobile.setUplinkLatency(2); // latency of connection between the smartphone and proxy server is 2 ms
            locator.linkDataWithInstance(mobile.getId(), mobileUserDataIds.get(i));
            mobile.setLevel(3);

            fogDevices1.add(mobile);
        }

    }

    private static FogDevice addMobile(String name, int userId, String appId, int parentId) {
        FogDevice mobile = createFogDevice1(name, 500, 10000, 180, 180, 0, 1.0, 0.000036, MicroserviceFogDevice.CLIENT);
        mobile.setParentId(parentId);
        //locator.setInitialLocation(name,drone.getId());
        Sensor mobileSensor = new Sensor("sensor-" + name, "M-SENSOR", userId, appId, new DeterministicDistribution(SENSOR_TRANSMISSION_TIME)); // inter-transmission time of EEG sensor follows a deterministic distribution
        mobileSensor.setApp(applications.get(0));
        sensors.add(mobileSensor);
        Actuator mobileDisplay = new Actuator("actuator-" + name, userId, appId, "M-DISPLAY");
        actuators.add(mobileDisplay);

        mobileSensor.setGatewayDeviceId(mobile.getId());
        mobileSensor.setLatency(6.0);  // latency of connection between EEG sensors and the parent Smartphone is 6 ms

        mobileDisplay.setGatewayDeviceId(mobile.getId());
        mobileDisplay.setLatency(1.0);  // latency of connection between Display actuator and the parent Smartphone is 1 ms
        mobileDisplay.setApp(applications.get(0));

        return mobile;
    }

	private static void createFogDevices(int userId, String appId, String datasetReference)throws IOException {
		FogDevice cloud = createFogDevice("cloud", 44800, 40000, 100, 10000, 0, 0.01, 16*103, 16*83.25);
		cloud.setParentId(-1);
		locator.linkDataWithInstance(cloud.getId(), locator.getLevelWiseResources(locator.getLevelID("Cloud")).get(0));
		fogDevices1.add(cloud);
		FogDevice proxy = createFogDevice("proxy-server", 2800, 4000, 10000, 10000, 1, 0.0, 107.339, 83.4333);
		proxy.setParentId(cloud.getId());
		locator.linkDataWithInstance(proxy.getId(), locator.getLevelWiseResources(locator.getLevelID("Proxy")).get(1));
		proxy.setUplinkLatency(100); // latency of connection between proxy server and cloud is 100 ms
		fogDevices1.add(proxy);
		for(int i=0;i<numOfAreas;i++){
			addArea(i+"", userId, appId, proxy.getId(), datasetReference);
		}
	}

	private static FogDevice addArea(String id, int userId, String appId, int parentId, String datasetReference)throws IOException{
		FogDevice router = createFogDevice("a-"+id, 2800, 10000, 180, 180, 2, 0.0, 1.0, 0.000036);
		fogDevices1.add(router);
		router.setUplinkLatency(2); // latency of connection between router and proxy server is 2 ms
		for(int i=0;i<numOfCamerasPerArea1;i++){
			userMobilityPattern.put(i, References.DIRECTIONAL_MOBILITY);
			String mobileId = id+"-"+i;
			FogDevice camera1 = addCamera1(mobileId, userId, appId, router.getId()); // adding a smart camera to the physical topology. Smart cameras have been modeled as fog devices as well.
			locator.parseUserInfo(userMobilityPattern, datasetReference);
			camera1.setUplinkLatency(2); // latency of connection between camera and router is 2 ms
			fogDevices1.add(camera1);
		}
		router.setParentId(parentId);
		return router;
	}
	
	private static FogDevice addCamera1(String id, int userId, String appId, int parentId){
		FogDevice camera1 = createFogDevice("m-"+id, 500, 10000, 180, 180, 3, 0, 1.0, 0.000036);
		camera1.setParentId(parentId);
		Sensor sensor = new Sensor("s-"+id, "CAMERA", userId, appId, new DeterministicDistribution(5)); // inter-transmission time of camera (sensor) follows a deterministic distribution
		sensors.add(sensor);
		Actuator ptz = new Actuator("ptz-"+id, userId, appId, "PTZ_CONTROL");
		actuators.add(ptz);
		sensor.setGatewayDeviceId(camera1.getId());
		sensor.setLatency(1.0);  // latency of connection between camera (sensor) and the parent Smart Camera is 1 ms
		ptz.setGatewayDeviceId(camera1.getId());
		ptz.setLatency(1.0);  // latency of connection between PTZ Control and the parent Smart Camera is 1 ms
		return camera1;
	}


	/**
	 * Creates a vanilla fog device
	 * @param nodeName name of the device to be used in simulation
	 * @param mips MIPS
	 * @param ram RAM
	 * @param upBw uplink bandwidth
	 * @param downBw downlink bandwidth
	 * @param level hierarchy level of the device
	 * @param ratePerMips cost rate per MIPS used
	 * @param busyPower
	 * @param idlePower
	 * @return
	 */
	private static FogDevice createFogDevice(String nodeName, long mips,
			int ram, long upBw, long downBw, int level, double ratePerMips, double busyPower, double idlePower) {
		
		List<Pe> peList = new ArrayList<Pe>();

		// 3. Create PEs and add these into a list.
		peList.add(new Pe(0, new PeProvisionerOverbooking(mips))); // need to store Pe id and MIPS Rating

		int hostId = FogUtils.generateEntityId();
		long storage = 1000; // host storage
		int bw = 180;

		PowerHost host = new PowerHost(
				hostId,
				new RamProvisionerSimple(ram),
				new BwProvisionerOverbooking(bw),
				storage,
				peList,
				new StreamOperatorScheduler(peList),
				new FogLinearPowerModel(busyPower, idlePower)
			);
		List<Host> hostList = new ArrayList<Host>();
		hostList.add(host);
		String arch = "x86"; // system architecture
		String os = "Linux"; // operating system
		String vmm = "Xen";
		double time_zone = 10.0; // time zone this resource located
		double cost = 3.0; // the cost of using processing in this resource
		double costPerMem = 0.05; // the cost of using memory in this resource
		double costPerStorage = 0.001; // the cost of using storage in this
										// resource
		double costPerBw = 0.0; // the cost of using bw in this resource
		LinkedList<Storage> storageList = new LinkedList<Storage>(); // we are not adding SAN
													// devices by now
		FogDeviceCharacteristics characteristics = new FogDeviceCharacteristics(
				arch, os, vmm, host, time_zone, cost, costPerMem,
				costPerStorage, costPerBw);
		FogDevice fogdevice = null;
		try {
			fogdevice = new FogDevice(nodeName, characteristics, 
					new AppModuleAllocationPolicy(hostList), storageList, 10, upBw, downBw, 0, ratePerMips);
		} catch (Exception e) {
			e.printStackTrace();
		}
		fogdevice.setLevel(level);
		return fogdevice;
	}

	
    private static MicroserviceFogDevice createFogDevice1(String nodeName, long mips,
            int ram, long upBw, long downBw, double ratePerMips, double busyPower, double idlePower, String deviceType) {

    	List<Pe> peList = new ArrayList<Pe>();

    	// 3. Create PEs and add these into a list.
    	peList.add(new Pe(0, new PeProvisionerOverbooking(mips))); // need to store Pe id and MIPS Rating

    	int hostId = FogUtils.generateEntityId();
    	long storage = 1000000; // host storage
    	int bw = 10000;
    	
    	PowerHost host = new PowerHost(
    			hostId,
    			new RamProvisionerSimple(ram),
    			new BwProvisionerOverbooking(bw),
    			storage,
    			peList,
    			new StreamOperatorScheduler(peList),
    			new FogLinearPowerModel(busyPower, idlePower)
    			);
    	
    	List<Host> hostList = new ArrayList<Host>();
    	hostList.add(host);
    	
    	String arch = "x86"; // system architecture
    	String os = "Linux"; // operating system
    	String vmm = "Xen";
    	double time_zone = 10.0; // time zone this resource located
    	double cost = 3.0; // the cost of using processing in this resource
    	double costPerMem = 0.05; // the cost of using memory in this resource
    	double costPerStorage = 0.001; // the cost of using storage in this
    	// resource
    	double costPerBw = 0.0; // the cost of using bw in this resource
    	LinkedList<Storage> storageList = new LinkedList<Storage>(); // we are not adding SAN
    	// devices by now
    	
    	FogDeviceCharacteristics characteristics = new FogDeviceCharacteristics(
    			arch, os, vmm, host, time_zone, cost, costPerMem,
    			costPerStorage, costPerBw);
    	
    	MicroserviceFogDevice fogdevice = null;
    	try {
    		fogdevice = new MicroserviceFogDevice(nodeName, characteristics,
    				new AppModuleAllocationPolicy(hostList), storageList, 10, upBw, downBw, 1250000, 0, ratePerMips, deviceType);
    	} catch (Exception e) {
    		e.printStackTrace();
    	}
    	
    	return fogdevice;
    }

	/**
	 * Function to create the Intelligent Surveillance application in the DDF model. 
	 * @param appId unique identifier of the application
	 * @param userId identifier of the user of the application
	 * @return
	 */
	@SuppressWarnings({"serial" })
	private static Application createApplication1(String appId, int userId){
		
		Application application = Application.createApplication(appId, userId);
		/*
		 * Adding modules (vertices) to the application model (directed graph)
		 */
		application.addAppModule("picture-capture", 10);
		application.addAppModule("slot-detector", 10);
        application.addAppModule("processingModule", 10); // adding module Concentration Calculator to the application model
        application.addAppModule("storageModule", 10); // adding module Connector to the application model
		
		/*
		 * Connecting the application modules (vertices) in the application model (directed graph) with edges
		 */
		application.addAppEdge("CAMERA", "picture-capture", 1000, 500, "CAMERA", Tuple.UP, AppEdge.SENSOR); // adding edge from CAMERA (sensor) to Motion Detector module carrying tuples of type CAMERA
		application.addAppEdge("picture-capture", "processingModule", 1000, 500, "processor",Tuple.UP, AppEdge.MODULE);
		application.addAppEdge("processingModule", "storageModule", 1000, 500, "storage",Tuple.UP, AppEdge.MODULE);
		application.addAppEdge("storageModule", "slot-detector", 1000, 500, "slots",Tuple.DOWN, AppEdge.MODULE);
		// adding edge from Slot Detector to PTZ CONTROL (actuator)
		application.addAppEdge("slot-detector2", "PTZ_CONTROL2", 100, 28, 100, "PTZ_PARAMS2", Tuple.DOWN, AppEdge.ACTUATOR);
		
		application.addTupleMapping("picture-capture", "CAMERA", "CAMERA", new FractionalSelectivity(1.0));
		application.addTupleMapping("processingModule", "picture-capture", "processor", new FractionalSelectivity(1.0));
		application.addTupleMapping("storageModule", "processingModule", "storage", new FractionalSelectivity(1.0));
		application.addTupleMapping("slot-detector", "storageModule", "slots", new FractionalSelectivity(1.0));
		application.addTupleMapping("PTZ_CONTROL", "slot-detector", "PTZ_PARAMS", new FractionalSelectivity(1.0));
		
		final AppLoop loop = new AppLoop(new ArrayList<String>() {{
			add("CAMERA");
			add("picture-capture");
			add("processingModule");
			add("storageModule");
			add("slot-detector");
			add("PTZ_CONTROL");
		}});
		
		List<AppLoop> loops = new ArrayList<AppLoop>() {{
			add(loop);
		}};
		
		application.setLoops(loops);
		return application;
		}
	}