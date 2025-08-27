plugins {
    id("java-library")
}
java {
    sourceCompatibility = JavaVersion.VERSION_11
    targetCompatibility = JavaVersion.VERSION_11
}

dependencies {
    // https://mvnrepository.com/artifact/org.ejml/ejml-core
    implementation(libs.ejml.core)

    // https://mvnrepository.com/artifact/org.apache.commons/commons-statistics-distribution
    implementation(libs.commons.statistics.distribution)
}